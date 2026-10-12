/* nmpc_shim.c: plain-C wrapper around the generated nonlinear-MPC solver
 * (hold-up). Same contract as cmpc_shim.c, with an nmpc_ prefix:
 *     nmpc_init()  nmpc_reset()  nmpc_solve(...)  nmpc_free()  nmpc_set_hu(...)
 * All acados types stay inside this file.
 *
 * Velocity form: the solver state is [theta, alpha, theta_dot, alpha_dot,
 * u_prev] and the output is tau = u_prev + du0 (clamped to +-NMPC_TAU_MAX).
 * The model is the exact zero-order-hold discretisation: the torque is held
 * constant for the tick.
 *
 * Real-time iteration scheme:
 *   cold start : after init or reset, the first solve starts every node from
 *                the measured state, with zero increments
 *   shift      : after each solve the plan moves one node forward, so the next
 *                iteration linearises around the previous plan
 *   non-finite : tau = 0, the solver is reset, and the next tick starts cold
 *
 * HP (the number of nodes) is fixed when the solver is generated. The
 * increments are frozen to zero from stage HU to HP; HU can be changed at run
 * time.
 */
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#include "acados_solver_ocp_nmpc_furuta_942b611a.h"
#include "acados_c/ocp_nlp_interface.h"

#define CAP_T   ocp_nmpc_furuta_942b611a_solver_capsule
#define FN(name) ocp_nmpc_furuta_942b611a_acados_##name

#define NXA  5      /* augmented state size  (x + u_prev)            */
#define NX   4       /* plant states                                  */
#define NN   40        /* horizon nodes (= HP, fixed at codegen time)   */
#define NU   1          /* control: du                                   */

#define NMPC_TAU_MAX     5   /* [N.m] final safety clamp (= u_prev bound) */
/* Torque-increment bounds on the free stages. Withheld in this public version
 * (0.0 placeholders). */
#define NMPC_DU_MIN      0.0
#define NMPC_DU_MAX      0.0
#define NMPC_HU_DEFAULT  35        /* control horizon applied at nmpc_init()    */

static CAP_T          *g_cap = NULL;
static ocp_nlp_config *g_cfg;
static ocp_nlp_dims   *g_dims;
static ocp_nlp_in     *g_in;
static ocp_nlp_out    *g_out;

static int g_hu_current = -1;   /* HU currently applied, -1 = none yet        */
static int g_cold       = 1;    /* 1 -> next solve seeds the iterate from xa0  */

int    nmpc_n_horizon(void) { return NN;  }
int    nmpc_n_states(void)  { return NX;  }
double nmpc_tau_max(void)   { return NMPC_TAU_MAX; }
int    nmpc_hu_current(void){ return g_hu_current; }

static double now_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1e6 + t.tv_nsec * 1e-3;
}

/* ── nmpc_set_hu ─────────────────────────────────────────────────────────
 * Stages [0, hu): du bounds [NMPC_DU_MIN, NMPC_DU_MAX]. Stages [hu, hp):
 * du frozen to 0. Both ranges are written every call, so hu can move up
 * or down at runtime. Call between ticks only, never mid-solve.
 * Returns 0 on success; -1 not initialised, -2 hp != NN, -3 hu out of range.
 * ────────────────────────────────────────────────────────────────────── */
int nmpc_set_hu(int hu, int hp)
{
    if (!g_cap) return -1;
    if (hp != NN) return -2;
    if (hu < 0 || hu > hp) return -3;

    double lo = NMPC_DU_MIN, hi = NMPC_DU_MAX, zero = 0.0;

    for (int k = 0; k < hu; k++)
    {
        ocp_nlp_constraints_model_set(g_cfg, g_dims, g_in, g_out, k, "lbu", &lo);
        ocp_nlp_constraints_model_set(g_cfg, g_dims, g_in, g_out, k, "ubu", &hi);
    }
    for (int k = hu; k < hp; k++)
    {
        ocp_nlp_constraints_model_set(g_cfg, g_dims, g_in, g_out, k, "lbu", &zero);
        ocp_nlp_constraints_model_set(g_cfg, g_dims, g_in, g_out, k, "ubu", &zero);
    }

    g_hu_current = hu;
    return 0;
}

/* returns 0 on success, nonzero acados status otherwise */
int nmpc_init(void)
{
    if (g_cap) return 0;
    g_cap = FN(create_capsule)();
    int st = FN(create)(g_cap);
    if (st) { FN(free_capsule)(g_cap); g_cap = NULL; return st; }

    g_cfg  = FN(get_nlp_config)(g_cap);
    g_dims = FN(get_nlp_dims)(g_cap);
    g_in   = FN(get_nlp_in)(g_cap);
    g_out  = FN(get_nlp_out)(g_cap);

    int hu_st = nmpc_set_hu(NMPC_HU_DEFAULT, NN);
    if (hu_st) { FN(free)(g_cap); FN(free_capsule)(g_cap); g_cap = NULL; return -100 + hu_st; }

    g_cold = 1;
    return 0;
}

/* cold start: wipe warm start + QP memory. Call after a mode switch / fault. */
int nmpc_reset(void)
{
    if (!g_cap) return -1;
    int st = FN(reset)(g_cap, 1, 1, 0, 1);
    if (st) return st;
    g_cold = 1;
    return nmpc_set_hu(g_hu_current >= 0 ? g_hu_current : NMPC_HU_DEFAULT, NN);
}

static void seed_iterate(const double xa0[NXA])
{
    double zero = 0.0;
    for (int k = 0; k <= NN; k++)
        ocp_nlp_out_set(g_cfg, g_dims, g_out, g_in, k, "x", (void *)xa0);
    for (int k = 0; k < NN; k++)
        ocp_nlp_out_set(g_cfg, g_dims, g_out, g_in, k, "u", &zero);
}

static void shift_iterate(void)
{
    double xb[NXA], ub;
    for (int k = 0; k < NN; k++)
    {
        ocp_nlp_out_get(g_cfg, g_dims, g_out, k + 1, "x", xb);
        ocp_nlp_out_set(g_cfg, g_dims, g_out, g_in, k, "x", xb);
    }
    for (int k = 0; k < NN - 1; k++)
    {
        ocp_nlp_out_get(g_cfg, g_dims, g_out, k + 1, "u", &ub);
        ocp_nlp_out_set(g_cfg, g_dims, g_out, g_in, k, "u", &ub);
    }
}

/*
 * nmpc_solve
 *   x        : 4 doubles  [theta, alpha, theta_dot, alpha_dot]
 *   u_prev   : torque actually applied last tick [N.m]
 *   tau_out  : optimal torque to apply next  = clamp(u_prev + du0)
 *   status   : acados status (0 = success)
 *   t_us     : wall time of the solve [microseconds]
 * returns the acados status as well (0 = OK).
 *
 * On a NONZERO status tau_out is still written (best available, clamped) but
 * the caller should treat it as a fault / fall back.
 */
int nmpc_solve(const double x[NX], double u_prev,
               double *tau_out, int *status, double *t_us)
{
    if (!g_cap) return -1;

    double x0[NXA];
    for (int i = 0; i < NX; i++) x0[i] = x[i];
    x0[NX] = u_prev;

    if (g_cold) { seed_iterate(x0); g_cold = 0; }

    ocp_nlp_constraints_model_set(g_cfg, g_dims, g_in, g_out, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(g_cfg, g_dims, g_in, g_out, 0, "ubx", x0);

    double t0 = now_us();
    int st = FN(solve)(g_cap);
    double t1 = now_us();

    double du0 = 0.0;
    ocp_nlp_out_get(g_cfg, g_dims, g_out, 0, "u", &du0);

    double tau = u_prev + du0;
    if (!isfinite(tau))
    {
        tau = 0.0;
        nmpc_reset();
    }
    else
    {
        shift_iterate();
    }
    if (tau >  NMPC_TAU_MAX) tau =  NMPC_TAU_MAX;
    if (tau < -NMPC_TAU_MAX) tau = -NMPC_TAU_MAX;

    *tau_out = tau;
    *status  = st;
    *t_us    = t1 - t0;
    return st;
}

void nmpc_free(void)
{
    if (!g_cap) return;
    FN(free)(g_cap);
    FN(free_capsule)(g_cap);
    g_cap = NULL;
    g_hu_current = -1;
    g_cold = 1;
}
