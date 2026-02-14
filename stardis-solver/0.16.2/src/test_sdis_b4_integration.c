/* Copyright (C) 2016-2025 |Meso|Star> (contact@meso-star.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>. */

/* Phase B-4: Full integration test (T-INT).
 *
 * Activates ALL path types simultaneously in a single scene to verify the
 * state machine's completeness and correctness.
 *
 * Test cases (from phase_b4_test_design.md, section T-INT):
 *   T-INT.3: Persistent wavefront vs depth-first, 64x64 spp=256
 *            >= 95% pixel statistical compatibility
 *   T-INT.6: Memory leak check after full cleanup
 *
 * Scene configuration:
 *   - Solid box interior (lambda=1.0, cp=800, rho=2700, delta=0.05)
 *   - Fluid exterior (T=400K, cp=1000, rho=1.2)
 *   - Interface: hc=20, emissivity=0.6 (activates radiation + convection)
 *   - Radiative environment: T=300K
 *   - External source: point at (0.5, 0.5, 3.0), power=50W
 *   - picard_order=2 (activates picardN recursive stack)
 *
 * This exercises:
 *   - RAD_TRACE (radiative path)
 *   - CND_DS (delta-sphere conduction)
 *   - BND_DISPATCH (boundary dispatch)
 *   - SS_REINJECT (solid/solid reinjection via enclosure query)
 *   - SF_REINJECT / SFN (picard1/picardN solid/fluid)
 *   - CNV (convective path)
 *   - EXT (external flux with shadow + diffuse bounce)
 *   - ENC_LOCATE (M10 point-in-enclosure)
 */

#include "sdis_solve_wavefront.h"
#include "sdis_solve_persistent_wavefront.h"
#include "test_sdis_b4_e2e_utils.h"

#include <stdio.h>
#include <string.h>

/* Integration test image parameters */
#define INT_IMG_WIDTH   64
#define INT_IMG_HEIGHT  64
#define INT_SPP         256
#define INT_PICARD      2

/* ========================================================================== */
/* T-INT.3: Persistent wavefront vs depth-first                               */
/* ========================================================================== */
static int
test_tint_3_e2e(void)
{
  struct sdis_device* dev = NULL;
  struct sdis_medium* solid = NULL;
  struct sdis_medium* fluid = NULL;
  struct sdis_interface* interf = NULL;
  struct sdis_radiative_env* radenv = NULL;
  struct sdis_source* source = NULL;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_scene* scn = NULL;
  struct sdis_camera* cam = NULL;
  struct sdis_estimator_buffer* img_ref = NULL;
  struct sdis_estimator_buffer* img_wf  = NULL;
  struct sdis_interface* interfaces[12];
  struct time t_ref, t_wf;
  int pass = 0;
  size_t n_cmp = 0, n_ok = 0;
  int i;
  const double src_pos[3] = {0.5, 0.5, 3.0};

  fprintf(stdout, "  T-INT.3: Full integration e2e (%dx%d spp=%d picard=%d)...\n",
    INT_IMG_WIDTH, INT_IMG_HEIGHT, INT_SPP, INT_PICARD);
  fflush(stdout);

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* Solid: unknown temperature, moderate conductivity */
  solid = e2e_create_solid(dev, SDIS_TEMPERATURE_NONE, 1.0,
                           800.0, 2700.0, 0.05, SDIS_VOLUMIC_POWER_NONE);

  /* Fluid: known temperature at 400K */
  fluid = e2e_create_fluid(dev, 400.0, 1000.0, 1.2);

  /* Interface: convection + radiation coupling
   *   hc = 20 W/(m^2*K) -> triggers convective path
   *   emissivity = 0.6   -> triggers radiative path + picardN */
  interf = e2e_create_interface(dev, fluid, solid, 20.0, 0.6, 0.0, 300.0);

  /* Radiative environment: 300K background */
  radenv = e2e_create_radenv(dev, 300.0, 300.0);

  /* External point source -> triggers EXT shadow ray + diffuse bounce */
  source = e2e_create_point_source(dev, src_pos, 50.0, 0.01);

  for(i = 0; i < 12; i++) interfaces[i] = interf;

  scn_args.get_indices   = box_get_indices;
  scn_args.get_position  = box_get_position;
  scn_args.get_interface = box_get_interface;
  scn_args.nprimitives   = box_ntriangles;
  scn_args.nvertices     = box_nvertices;
  scn_args.t_range[0]    = 200;
  scn_args.t_range[1]    = 500;
  scn_args.radenv        = radenv;
  scn_args.source        = source;
  scn_args.context       = interfaces;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  cam = e2e_create_camera(dev, INT_IMG_WIDTH, INT_IMG_HEIGHT);

  /* ---------- Depth-first reference ---------- */
  fprintf(stdout, "    Depth-first run...\n");
  OK(e2e_run_solve_camera(scn, cam, 0,
    INT_IMG_WIDTH, INT_IMG_HEIGHT, INT_SPP, INT_PICARD,
    SDIS_DIFFUSION_DELTA_SPHERE, &img_ref, &t_ref));
  {
    char ts[128];
    time_dump(&t_ref, TIME_ALL, NULL, ts, sizeof(ts));
    fprintf(stdout, "    Depth-first time: %s\n", ts);
  }

  /* ---------- Wavefront ---------- */
  fprintf(stdout, "    Wavefront run...\n");
  OK(e2e_run_solve_camera(scn, cam, 1,
    INT_IMG_WIDTH, INT_IMG_HEIGHT, INT_SPP, INT_PICARD,
    SDIS_DIFFUSION_DELTA_SPHERE, &img_wf, &t_wf));
  {
    char ts[128];
    time_dump(&t_wf, TIME_ALL, NULL, ts, sizeof(ts));
    fprintf(stdout, "    Wavefront time: %s\n", ts);
  }

  /* ---------- T-INT.3: Comparison ---------- */
  fprintf(stdout, "    Comparison:\n");
  e2e_compare_images(img_ref, img_wf, E2E_TOL_SIGMA, E2E_PASS_RATE,
                     &pass, &n_cmp, &n_ok);

  /* ---------- Speedup ---------- */
  {
    int64_t usec_ref = time_val(&t_ref, TIME_USEC);
    int64_t usec_wf  = time_val(&t_wf,  TIME_USEC);
    if(usec_wf > 0) {
      double speedup = (double)usec_ref / (double)usec_wf;
      fprintf(stdout, "    Speedup: %.2fx", speedup);
      if(speedup < 0.9) {
        fprintf(stdout, " [WARNING: wavefront slower than depth-first]");
      }
      fprintf(stdout, "\n");
    }
  }

  /* ---------- Cleanup ---------- */
  OK(sdis_estimator_buffer_ref_put(img_ref));
  OK(sdis_estimator_buffer_ref_put(img_wf));
  OK(sdis_camera_ref_put(cam));
  OK(sdis_scene_ref_put(scn));
  OK(sdis_source_ref_put(source));
  OK(sdis_radiative_env_ref_put(radenv));
  OK(sdis_interface_ref_put(interf));
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(fluid));
  free_default_device(dev);

  return pass;
}

/* ========================================================================== */
/* T-INT.6: Memory leak check                                                 */
/* ========================================================================== */
static void
test_tint_6_memory_leak(void)
{
  printf("  T-INT.6: Memory leak check... ");
  CHK(mem_allocated_size() == 0);
  printf("PASS (0 bytes leaked)\n");
}

/* ========================================================================== */
/* main                                                                       */
/* ========================================================================== */
int
main(int argc, char** argv)
{
  int pass_e2e;
  (void)argc; (void)argv;

  fprintf(stdout,
    "=== Phase B-4: Full Integration Test (T-INT) ===\n"
    "Scene: unit box, solid+fluid coupling, external source\n"
    "Physics: conduction + convection + radiation + external flux\n"
    "Picard order: %d, Image: %dx%d, SPP: %d\n\n",
    INT_PICARD, INT_IMG_WIDTH, INT_IMG_HEIGHT, INT_SPP);

  /* T-INT.3: end-to-end comparison */
  pass_e2e = test_tint_3_e2e();

  /* T-INT.6: memory leak check (after all scene objects released) */
  test_tint_6_memory_leak();

  fprintf(stdout, "\n=== Integration Test: %s ===\n",
    pass_e2e ? "PASS" : "FAIL");

  return pass_e2e ? 0 : 1;
}
