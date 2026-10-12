/* cmpc_shim.c: plain-C wrapper around the generated constrained-MPC solver.
 *
 * Python (through ctypes) sees:
 *     cmpc_init()  cmpc_reset()  cmpc_solve(...)  cmpc_free()  cmpc_set_hu(...)
 * All acados types stay inside this file.
 *
 * Velocity form: the solver state is [theta, alpha, theta_dot, alpha_dot,
 * u_prev] and it optimises torque increments, so the output is
 *     tau = u_prev + du0      (clamped to +-CMPC_TAU_MAX)
 *
 * Horizons. HP (the number of prediction nodes) is fixed when the solver is
 * generated. HU, the control horizon, can be changed at run time: the
 * increments are forced to zero from stage HU to HP, so the torque is held
 * constant over that tail.
 */
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#include "acados_solver_ocp_cmpc_furuta_2b91e330.h"
#include "acados_c/ocp_nlp_interface.h"

#define CAP_T   ocp_cmpc_furuta_2b91e330_solver_capsule
#define FN(name) ocp_cmpc_furuta_2b91e330_acados_##name

#define NXA  5      /* augmented state size  (x + u_prev)            */
#define NX   4       /* plant states                                  */
#define NN   10        /* horizon nodes (= HP, fixed at codegen time)   */
#define NBU  1          /* number of bounded controls (du); model has nu=1 */

#define CMPC_TAU_MAX     5   /* [N.m] final safety clamp (= U bound) */
#define CMPC_HU_DEFAULT  4        /* control horizon applied at cmpc_init() */

static CAP_T        *g_cap = NULL;
static ocp_nlp_config *g_cfg;
static ocp_nlp_dims   *g_dims;
static ocp_nlp_in     *g_in;
static ocp_nlp_out    *g_out;
static ocp_nlp_solver *g_solver;

static int g_hu_current = -1;   /* HU currently applied to the solver, -1 = none yet */

/* ── info exported so Python can check it loaded the solver it expects ───── */
int    cmpc_n_horizon(void) { return NN;  }
int    cmpc_n_states(void)  { return NX;  }
double cmpc_tau_max(void)   { return CMPC_TAU_MAX; }
int    cmpc_hu_current(void){ return g_hu_current; }

static double now_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1e6 + t.tv_nsec * 1e-3;
}

/* cmpc_set_hu: forces du = 0 on stages [hu, hp), so the torque is held
 * constant from stage hu onward. hp must equal NN (fixed when the solver is
 * generated); only hu can change at run time. Call between ticks only, never
 * during a solve.
 *
 * Returns 0 on success, -1 if the solver is not initialised, -2 if hp does
 * not match NN, -3 if hu is outside [0, hp]. */
int cmpc_set_hu(int hu, int hp)
{
    if (!g_cap) return -1;
    if (hp != NN) return -2;
    if (hu < 0 || hu > hp) return -3;

    double zero[NBU];
    for (int i = 0; i < NBU; i++) zero[i] = 0.0;

    /* Only freezes stages. Unfreezing would need the original increment bounds,
     * which this shim does not keep, so hu should only move once, at start-up. */

    for (int k = hu; k < hp; k++)
    {
        ocp_nlp_constraints_model_set(g_cfg, g_dims, g_in, g_out, k, "lbu", zero);
        ocp_nlp_constraints_model_set(g_cfg, g_dims, g_in, g_out, k, "ubu", zero);
    }

    g_hu_current = hu;
    return 0;
}

/* returns 0 on success, nonzero acados status otherwise */
int cmpc_init(void)
{
    if (g_cap) return 0;
    g_cap = FN(create_capsule)();
    int st = FN(create)(g_cap);
    if (st) { FN(free_capsule)(g_cap); g_cap = NULL; return st; }

    g_cfg    = FN(get_nlp_config)(g_cap);
    g_dims   = FN(get_nlp_dims)(g_cap);
    g_in     = FN(get_nlp_in)(g_cap);
    g_out    = FN(get_nlp_out)(g_cap);
    g_solver = FN(get_nlp_solver)(g_cap);

    /* Apply the control horizon right away; otherwise every stage's increment
     * would be free (HU == HP). */
    int hu_st = cmpc_set_hu(CMPC_HU_DEFAULT, NN);
    if (hu_st) { FN(free)(g_cap); FN(free_capsule)(g_cap); g_cap = NULL; return -100 + hu_st; }

    return 0;
}

/* cold start: wipe warm-start + QP memory. Call after a mode switch / fault. */
int cmpc_reset(void)
{
    if (!g_cap) return -1;
    /* qp mem, numerical values, keep solver options, x -> x0 bar */
    int st = FN(reset)(g_cap, 1, 1, 0, 1);
    if (st) return st;

    /* acados_reset() re-initializes NLP memory; re-apply the HU freeze so
     * a reset can never silently drop back to HU == HP. */
    return cmpc_set_hu(g_hu_current >= 0 ? g_hu_current : CMPC_HU_DEFAULT, NN);
}

/*
 * cmpc_solve
 *   x        : 4 doubles  [theta, alpha, theta_dot, alpha_dot]
 *   u_prev   : torque actually applied last tick [N.m]
 *   tau_out  : optimal torque to apply next  = clamp(u_prev + du0)
 *   status   : acados status (0 = success)
 *   t_us     : wall time of the solve [microseconds]
 * returns the acados status as well (0 = OK).
 *
 * On a NONZERO status tau_out is still written (best available, clamped) but
 * the caller (the Pi main loop) should treat it as a fault / fall back.
 */
int cmpc_solve(const double x[NX], double u_prev,
               double *tau_out, int *status, double *t_us)
{
    if (!g_cap) return -1;

    double x0[NXA];
    for (int i = 0; i < NX; i++) x0[i] = x[i];
    x0[NX] = u_prev;

    /* fix the initial node: lbx = ubx = x0  (all NXA entries) */
    ocp_nlp_constraints_model_set(g_cfg, g_dims, g_in, g_out, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(g_cfg, g_dims, g_in, g_out, 0, "ubx", x0);

    double t0 = now_us();
    int st = FN(solve)(g_cap);
    double t1 = now_us();

    double du0 = 0.0;
    ocp_nlp_out_get(g_cfg, g_dims, g_out, 0, "u", &du0);

    double tau = u_prev + du0;
    if (!isfinite(tau)) tau = 0.0;
    if (tau >  CMPC_TAU_MAX) tau =  CMPC_TAU_MAX;
    if (tau < -CMPC_TAU_MAX) tau = -CMPC_TAU_MAX;

    *tau_out = tau;
    *status  = st;
    *t_us    = t1 - t0;
    return st;
}

void cmpc_free(void)
{
    if (!g_cap) return;
    FN(free)(g_cap);
    FN(free_capsule)(g_cap);
    g_cap = NULL;
    g_hu_current = -1;
}
