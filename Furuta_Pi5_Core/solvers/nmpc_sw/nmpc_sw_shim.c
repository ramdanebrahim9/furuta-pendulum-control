/* nmpc_sw_shim.c: plain-C wrapper around the generated swing-up NMPC solver.
 *
 * Python (through ctypes) sees:
 *     nmpc_sw_init()  nmpc_sw_reset()  nmpc_sw_solve(...)  nmpc_sw_predict(...)  nmpc_sw_free()
 * All acados types stay inside this file.
 *
 * Plain-torque form: the state is [theta, alpha, theta_dot, alpha_dot] and the
 * control is the torque itself. Swing-up needs torque reversals, so there is
 * no previous-torque state and no increment penalty (unlike the hold-up MPCs).
 *
 * Cold start. After a reset the first solve must not start from a flat guess
 * (pendulum at rest, zero torque): that point is a symmetric equilibrium of
 * the cost, so the solver finds no slope to follow and may need hundreds of
 * iterations. Instead the first guess is a rollout of the energy-based
 * swing-up law (see Controller_Energy_SwingUp). It does not have to be a good
 * trajectory, only a moving one. The caller decides how many solver
 * iterations to run on that first tick, and one per tick after that.
 *
 * Output: tau = clamp(u[0], +-NMPC_SW_TAU_MAX). A non-finite result gives
 * tau = 0 and forces the next solve to start cold.
 */
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#include "acados_solver_ocp_nmpc_sw_furuta_c82cf6f7.h"
#include "acados_c/ocp_nlp_interface.h"
#include "furuta_f4_seed.h"   /* CasADi-generated plant, cold-start seed only */

#define CAP_T   ocp_nmpc_sw_furuta_c82cf6f7_solver_capsule
#define FN(name) ocp_nmpc_sw_furuta_c82cf6f7_acados_##name

#define NX   4       /* plant states: theta, alpha, theta_dot, alpha_dot */
#define NN   70         /* horizon nodes (= N_FINE + N_COARSE, fixed)       */
#define NU   1           /* control: tau                                    */

#define N_FINE    70
#define DT_FINE   0.0050000000000000001
#define N_COARSE  0
#define DT_COARSE 0.040000000000000001
/* Time step after node k: a fine grid, then a coarse tail. */
#define NODE_DT(k)  ((k) < N_FINE ? DT_FINE : DT_COARSE)

/* Withheld in this public version (0.0 placeholders). */
#define NMPC_SW_TAU_MAX  0.0   /* [N.m] swing-up torque bound */
#define NMPC_SW_COLD_ITERS_DEFAULT  35   /* solver iterations suggested for the first (cold) tick */

/* Cold-start energy-pump gains, used only to build the first guess for the
 * solver; the rollout is never applied to the motor. Withheld in this public
 * version (0.0 placeholders). */
#define COLD_K_ENERGY      0.0
#define COLD_TAU_PUMP      0.0
#define COLD_SOFT_WIDTH    0.0
#define COLD_K_ARM_P       0.0
#define COLD_K_ARM_D       0.0
#define COLD_RK4_SUBSTEP   0.0025   /* [s] integration sub-step of the rollout */

/* The energy law needs a few plant parameters; the plant itself comes from
 * furuta_f4_seed.c, generated from the same physics as the solver's model.
 * Withheld in this public version (placeholders). */
#define P_JP  0.0
#define P_MP  0.0
#define P_LC  0.0
#define P_G   9.81

static CAP_T          *g_cap = NULL;
static ocp_nlp_config *g_cfg;
static ocp_nlp_dims   *g_dims;
static ocp_nlp_in     *g_in;
static ocp_nlp_out    *g_out;

static int g_cold = 1;    /* 1 -> next solve seeds the iterate from x0 first */

/* The plan acados computed on the LAST nmpc_sw_solve(), captured BEFORE
 * shift_iterate() moves it forward (afterwards node 0 holds what used to be
 * node 1). Exists so the Pi's per-tick log can put "the torque we sent" next
 * to "what acados planned": u0_raw is the unclamped optimizer output,
 * u1 / x1 the next planned torque and the state it predicts one fine step
 * (DT_FINE) after x0. Read via nmpc_sw_last_plan(). */
static double g_last_u0 = 0.0, g_last_u1 = 0.0, g_last_x1[NX] = {0.0, 0.0, 0.0, 0.0};

int    nmpc_sw_n_horizon(void) { return NN; }
int    nmpc_sw_n_states(void)  { return NX; }
double nmpc_sw_tau_max(void)   { return NMPC_SW_TAU_MAX; }
/* Suggested number of solver iterations for the first (cold) tick. Only a
 * suggestion: nmpc_sw_solve() always runs exactly the iterations it is given. */
int    nmpc_sw_cold_iters_default(void) { return NMPC_SW_COLD_ITERS_DEFAULT; }

static double now_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1e6 + t.tv_nsec * 1e-3;
}

int nmpc_sw_init(void)
{
    if (g_cap) return 0;
    g_cap = FN(create_capsule)();
    int st = FN(create)(g_cap);
    if (st) { FN(free_capsule)(g_cap); g_cap = NULL; return st; }

    g_cfg  = FN(get_nlp_config)(g_cap);
    g_dims = FN(get_nlp_dims)(g_cap);
    g_in   = FN(get_nlp_in)(g_cap);
    g_out  = FN(get_nlp_out)(g_cap);

    g_cold = 1;
    return 0;
}

/* Cold start: wipe warm-start + QP memory. Call once, on the *_Swingup_Reset
 * sentinel tick, BEFORE the caller starts stacking nmpc_sw_solve() calls. */
int nmpc_sw_reset(void)
{
    if (!g_cap) return -1;
    int st = FN(reset)(g_cap, 1, 1, 0, 1);
    if (st) return st;
    g_cold = 1;
    return 0;
}

/* Plant dynamics xdot = f(x, tau), generated from the same symbolic model as
 * the solver (furuta_f4_seed.c). Used only to build the cold-start guess; the
 * solver's own dynamics live in the generated code. */
static void furuta_f4_step(const double x[NX], double tau, double xdot[NX])
{
    double w[furuta_f4_SZ_W];
    long long iw[1];               /* furuta_f4_SZ_IW is 0; 1 avoids a zero-length array */
    const double *arg[2] = { x, &tau };
    double *res[1] = { xdot };
    furuta_f4(arg, res, iw, w, 0);
}

/* One RK4 step of length h with the torque held constant. */
static void rk4_step(double x[NX], double tau, double h)
{
    double k1[NX], k2[NX], k3[NX], k4[NX], xt[NX];
    furuta_f4_step(x, tau, k1);
    for (int i = 0; i < NX; i++) xt[i] = x[i] + 0.5*h*k1[i];
    furuta_f4_step(xt, tau, k2);
    for (int i = 0; i < NX; i++) xt[i] = x[i] + 0.5*h*k2[i];
    furuta_f4_step(xt, tau, k3);
    for (int i = 0; i < NX; i++) xt[i] = x[i] + h*k3[i];
    furuta_f4_step(xt, tau, k4);
    for (int i = 0; i < NX; i++)
        x[i] += (h/6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
}

/* The energy-based swing-up law (Astrom & Furuta) with arm centering, used
 * only to build the cold-start guess. */
static double swingup_law(double theta, double al, double al_dot, double th_dot)
{
    double Jp_pivot = P_JP + P_MP*P_LC*P_LC;
    double mgl      = P_MP*P_G*P_LC;
    double E        = 0.5*Jp_pivot*al_dot*al_dot + mgl*(cos(al) - 1.0);
    double E_err    = E - 0.0;                         /* E0 = 0 at upright */

    double dir = -tanh( (al_dot * cos(al)) / COLD_SOFT_WIDTH );
    double tau = COLD_K_ENERGY * E_err * dir;

    if (tau >  COLD_TAU_PUMP) tau =  COLD_TAU_PUMP;
    if (tau < -COLD_TAU_PUMP) tau = -COLD_TAU_PUMP;

    tau -= COLD_K_ARM_P*theta + COLD_K_ARM_D*th_dot;

    if (tau >  NMPC_SW_TAU_MAX) tau =  NMPC_SW_TAU_MAX;
    if (tau < -NMPC_SW_TAU_MAX) tau = -NMPC_SW_TAU_MAX;
    return tau;
}

/* seed every horizon node with an energy-pump rollout starting at x0,
 * stepped at the solver's own fine-grid spacing (0.0050000000000000001 s) so the guess
 * lines up with the fine head of the non-uniform grid; the coarse tail gets
 * the rollout value at its own node time (not re-sub-stepped -- same
 * approximation shift_iterate() already makes on the coarse tail). */
static void seed_iterate(const double x0[NX])
{
    double x[NX] = { x0[0], x0[1], x0[2], x0[3] };
    ocp_nlp_out_set(g_cfg, g_dims, g_out, g_in, 0, "x", (void *)x);

    for (int k = 0; k < NN; k++)
    {
        double tau = swingup_law(x[0], x[1], x[3], x[2]);
        ocp_nlp_out_set(g_cfg, g_dims, g_out, g_in, k, "u", &tau);

        int n_sub = (int)(NODE_DT(k) / COLD_RK4_SUBSTEP + 0.999999);
        if (n_sub < 1) n_sub = 1;
        double h = NODE_DT(k) / n_sub;
        for (int s = 0; s < n_sub; s++) rk4_step(x, tau, h);

        ocp_nlp_out_set(g_cfg, g_dims, g_out, g_in, k + 1, "x", (void *)x);
    }
}

/* Moves the plan one node forward so the next solve starts from it. Exact on
 * the fine grid and an approximation on the coarse tail, which the next
 * solve corrects. */
static void shift_iterate(void)
{
    double xb[NX], ub;
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

/* The STM32 sends alpha wrapped to [-pi, pi], but the plan is continuous.
 * Before a warm solve, the whole plan's alpha is shifted by a multiple of
 * 2*pi so that its first node sits next to the new measurement. The cost is
 * wrap-safe, but the dynamics constraint between nodes is not: without this
 * shift, the tick where alpha crosses +-pi would leave a 2*pi gap between the
 * first two nodes and the solver would stall. */
#define NMPC_SW_TWO_PI 6.283185307179586476925286766559
static void align_iterate_alpha(const double x0[NX])
{
    double xb[NX];
    ocp_nlp_out_get(g_cfg, g_dims, g_out, 0, "x", xb);
    double k = round((x0[1] - xb[1]) / NMPC_SW_TWO_PI);
    if (k == 0.0) return;
    for (int n = 0; n <= NN; n++)
    {
        ocp_nlp_out_get(g_cfg, g_dims, g_out, n, "x", xb);
        xb[1] += NMPC_SW_TWO_PI * k;
        ocp_nlp_out_set(g_cfg, g_dims, g_out, g_in, n, "x", xb);
    }
}

/*
 * nmpc_sw_predict: integrates the plant from x for dt seconds with the torque
 * held constant, using RK4 sub-steps on the same model as the solver.
 * Used by Controller_NMPC_SW:
 *   cold tick : predict(state at reset, 0, T_HOLD)   the STM32 holds zero
 *               torque for T_HOLD, so plan from where the pendulum will be then
 *   warm tick : predict(state, torque sent, 5 ms)    one-tick delay
 * x_out may alias x. Returns 0, or -1 if dt < 0.
 */
int nmpc_sw_predict(const double x[NX], double tau, double dt, double x_out[NX])
{
    if (dt < 0.0) return -1;
    double xs[NX] = { x[0], x[1], x[2], x[3] };
    if (dt > 0.0)
    {
        int n_sub = (int)(dt / COLD_RK4_SUBSTEP + 0.999999);
        if (n_sub < 1) n_sub = 1;
        double h = dt / n_sub;
        for (int s = 0; s < n_sub; s++) rk4_step(xs, tau, h);
    }
    for (int i = 0; i < NX; i++) x_out[i] = xs[i];
    return 0;
}

/*
 * nmpc_sw_solve
 *   x       : [theta, alpha, theta_dot, alpha_dot], already predicted for the
 *             delay (see nmpc_sw_predict(); this function never predicts)
 *   iters   : solver iterations on this same x: 1 on normal ticks, more only
 *             on the cold tick right after nmpc_sw_reset()
 *   tau_out : torque to apply, clamp(u[0], +-NMPC_SW_TAU_MAX)
 *   status  : status of the last iteration (0 = success)
 *   t_us    : total time over all iterations [microseconds]
 * Returns the same status as *status.
 */
int nmpc_sw_solve(const double x[NX], int iters,
                   double *tau_out, int *status, double *t_us)
{
    if (!g_cap) return -1;
    if (iters < 1) iters = 1;

    if (g_cold) seed_iterate(x);
    else        align_iterate_alpha(x);

    ocp_nlp_constraints_model_set(g_cfg, g_dims, g_in, g_out, 0, "lbx", (void *)x);
    ocp_nlp_constraints_model_set(g_cfg, g_dims, g_in, g_out, 0, "ubx", (void *)x);

    double t0 = now_us();
    int st = 0;
    for (int it = 0; it < iters; it++)
        st = FN(solve)(g_cap);
    double t1 = now_us();

    double u0 = 0.0;
    ocp_nlp_out_get(g_cfg, g_dims, g_out, 0, "u", &u0);
    g_last_u0 = u0;
    ocp_nlp_out_get(g_cfg, g_dims, g_out, 1, "u", &g_last_u1);
    ocp_nlp_out_get(g_cfg, g_dims, g_out, 1, "x", g_last_x1);

    double tau = u0;
    if (!isfinite(tau))
    {
        tau = 0.0;
        nmpc_sw_reset();
    }
    else
    {
        g_cold = 0;
        shift_iterate();
    }
    if (tau >  NMPC_SW_TAU_MAX) tau =  NMPC_SW_TAU_MAX;
    if (tau < -NMPC_SW_TAU_MAX) tau = -NMPC_SW_TAU_MAX;

    *tau_out = tau;
    *status  = st;
    *t_us    = t1 - t0;
    return st;
}

/* The plan captured by the last nmpc_sw_solve() (see g_last_u0 above). */
int nmpc_sw_last_plan(double *u0_raw, double *u1, double x1[NX])
{
    *u0_raw = g_last_u0;
    *u1     = g_last_u1;
    for (int i = 0; i < NX; i++) x1[i] = g_last_x1[i];
    return 0;
}

void nmpc_sw_free(void)
{
    if (!g_cap) return;
    FN(free)(g_cap);
    FN(free_capsule)(g_cap);
    g_cap = NULL;
    g_cold = 1;
}
