/* P0 Wavefront numerical correctness test utilities.
 *
 * Provides shared configuration, comparison helpers, and sweep routines
 * for the P0-level tests (WF-A1, WF-A2, WF-B2, WF-C1, WF-D1).
 *
 * Validation protocol (per wf_p0_test_design.md v3):
 *   Primary  — wavefront result vs hardcoded analytic temperature
 *              Decides PASS/FAIL.
 *   Diagnostic — wavefront vs depth-first (same cuBQL backend)
 *              Logs only, does NOT affect test outcome.
 *
 * All P0 tests link sdis_obj (OBJECT lib) for access to LOCAL_SYM symbols.
 */

#ifndef TEST_SDIS_WF_P0_UTILS_H
#define TEST_SDIS_WF_P0_UTILS_H

#include "sdis.h"
#include "test_sdis_utils.h"
#include "test_sdis_b4_e2e_utils.h"

#include <math.h>
#include <stdio.h>
#include <float.h>

/* ========================================================================== */
/* Configuration                                                              */
/* ========================================================================== */
#define P0_NREALISATIONS  10000  /* per-probe realisation count               */
#define P0_NPROBES        11     /* standard X-axis sweep (x=0.0 .. 1.0)     */
#define P0_TOL_SIGMA      3.0   /* Primary: analytic tolerance (3 sigma)     */
#define P0_DIAG_SIGMA     4.0   /* Diagnostic: state-machine equiv (4 sigma) */
#define P0_PASS_RATE      0.95  /* 95% of probes must pass (Primary only)    */
#define P0_ENABLE_DIAG    0     /* 1=run Diagnostic, 0=skip to speed up      */

/* ========================================================================== */
/* Primary: analytic comparison (decides PASS/FAIL)                           */
/* Returns 1 = pass, 0 = fail.                                                */
/* ========================================================================== */
static int
p0_compare_analytic(
  struct sdis_estimator* est,
  double expected_T,
  double tol_sigma)
{
  struct sdis_mc mc;
  if(sdis_estimator_get_temperature(est, &mc) != RES_OK) return 0;
  if(mc.SE < 1e-15) return fabs(mc.E - expected_T) < 1e-10;
  return fabs(mc.E - expected_T) <= tol_sigma * mc.SE;
}

/* ========================================================================== */
/* Diagnostic: state-machine equivalence (log only, non-blocking)             */
/* Returns 1 = consistent, 0 = inconsistent.                                  */
/* ========================================================================== */
static int
p0_diag_compare(
  struct sdis_estimator* est_wf,
  struct sdis_estimator* est_df,
  double tol_sigma)
{
  struct sdis_mc mc_wf, mc_df;
  double sigma_comb;

  if(sdis_estimator_get_temperature(est_wf, &mc_wf) != RES_OK) return 0;
  if(sdis_estimator_get_temperature(est_df, &mc_df) != RES_OK) return 0;

  sigma_comb = sqrt(mc_wf.SE * mc_wf.SE + mc_df.SE * mc_df.SE);
  if(sigma_comb < 1e-15) return fabs(mc_wf.E - mc_df.E) < 1e-10;
  return fabs(mc_wf.E - mc_df.E) <= tol_sigma * sigma_comb;
}

/* ========================================================================== */
/* Diagnostic print for a single probe                                        */
/* ========================================================================== */
static void
p0_print_probe_result(
  double x,
  struct sdis_estimator* est_wf,
  struct sdis_estimator* est_df,   /* may be NULL when P0_ENABLE_DIAG == 0 */
  double T_ref)
{
  struct sdis_mc mc_wf = {0,0,0}, mc_df = {0,0,0};
  (void)sdis_estimator_get_temperature(est_wf, &mc_wf);
  if(est_df)
    (void)sdis_estimator_get_temperature(est_df, &mc_df);

  fprintf(stdout,
    "  x=%.3f  wf=%.4f (SE=%.2e)  ref=%.4f  primary=%.1f sigma",
    x, mc_wf.E, mc_wf.SE, T_ref,
    mc_wf.SE > 0 ? fabs(mc_wf.E - T_ref) / mc_wf.SE : 0.0);

  if(est_df) {
    double sc = sqrt(mc_wf.SE*mc_wf.SE + mc_df.SE*mc_df.SE);
    fprintf(stdout, "  df=%.4f  diag=%.1f sigma",
      mc_df.E,
      sc > 0 ? fabs(mc_wf.E - mc_df.E) / sc : 0.0);
  }
  fprintf(stdout, "\n");
}

/* ========================================================================== */
/* One-shot probe sweep: N probes along X axis                                */
/*   Primary (analytic) decides PASS/FAIL.                                    */
/*   Diagnostic (depth-first) is logged but non-blocking.                     */
/* Returns 1 = pass, 0 = fail.                                                */
/* ========================================================================== */
typedef double (*p0_analytic_fn)(double x);

static int
p0_run_probe_sweep(
  struct sdis_scene*    scn,
  p0_analytic_fn        T_analytic,
  size_t                nprobes,
  size_t                nrealisations,
  size_t                picard_order,
  enum sdis_diffusion_algorithm diff_algo,
  double                y_fixed,
  double                z_fixed)
{
  size_t i;
  int n_pass_primary = 0, n_pass_diag = 0;

  fprintf(stdout, "  Running %lu probes, %lu realisations each ...\n",
    (unsigned long)nprobes, (unsigned long)nrealisations);

  for(i = 0; i < nprobes; i++) {
    /* Offset probes inward to avoid degenerate behaviour exactly on
     * boundary surfaces.  x ranges from ~0.05 to ~0.95. */
    double x = 0.05 + (double)i * 0.9 / (double)(nprobes - 1);
    double pos[3];
    double T_ref;
    struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
    struct sdis_estimator *est_wf = NULL, *est_df = NULL;

    pos[0] = x; pos[1] = y_fixed; pos[2] = z_fixed;
    T_ref = T_analytic(x);

    args.nrealisations = nrealisations;
    args.position[0] = pos[0];
    args.position[1] = pos[1];
    args.position[2] = pos[2];
    args.picard_order = picard_order;
    args.diff_algo = diff_algo;

    /* Wavefront probe solve */
    OK(sdis_solve_wavefront_probe(scn, &args, &est_wf));

    /* Primary: wavefront vs analytic (decides PASS/FAIL) */
    n_pass_primary += p0_compare_analytic(est_wf, T_ref, P0_TOL_SIGMA);

    /* Diagnostic: wavefront vs depth-first (log only, non-blocking) */
    if(P0_ENABLE_DIAG) {
      OK(sdis_solve_probe(scn, &args, &est_df));
      n_pass_diag += p0_diag_compare(est_wf, est_df, P0_DIAG_SIGMA);
    }

    /* Diagnostic output */
    p0_print_probe_result(x, est_wf, est_df, T_ref);

    OK(sdis_estimator_ref_put(est_wf));
    if(est_df)
      OK(sdis_estimator_ref_put(est_df));
  }

  fprintf(stdout, "  Primary:    %d/%lu probes pass (%.1f%%)\n",
    n_pass_primary, (unsigned long)nprobes,
    100.0 * (double)n_pass_primary / (double)nprobes);

  if(P0_ENABLE_DIAG)
    fprintf(stdout, "  Diagnostic: %d/%lu probes consistent (%.1f%%)\n",
      n_pass_diag, (unsigned long)nprobes,
      100.0 * (double)n_pass_diag / (double)nprobes);

  /* Only Primary decides PASS/FAIL */
  return (double)n_pass_primary / (double)nprobes >= P0_PASS_RATE;
}

#endif /* TEST_SDIS_WF_P0_UTILS_H */
