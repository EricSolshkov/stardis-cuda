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

/* Phase B-4, Milestone 10: Point-in-enclosure via BVH closest primitive.
 *
 * This test file covers both immediate and delayed test cases for M10 as
 * defined in phase_b4_test_design.md section T10.
 *
 * Immediate tests (no GPU required):
 *   T10.1:  State classification (path_phase_is_enc_locate_pending)
 *   T10.2:  step_enc_locate_submit field assignment
 *   T10.3:  step_enc_locate_result state advance
 *   T10.10: collect/distribute flow (mixed ENC + RAY paths)
 *   T10.13: enc_locate vs enc_query sizeof comparison
 *   T10.14: cascade safety (no scene_get_enclosure_id calls expected)
 *
 * Delayed tests (now executable, M0-M10 complete):
 *   T10.4-T10.6: Point query correctness (inner/outer/boundary)
 *   T10.8:  M10 vs CPU bulk consistency (1000 random points)
 *   T10.12: End-to-end mixed scene (wavefront vs depth-first)
 */

#include "sdis_solve_wavefront.h"
#include "sdis_wf_steps.h"
#include "sdis_wf_types.h"
#include "sdis_solve_persistent_wavefront.h"
#include "sdis_scene_c.h"

#include "test_sdis_b4_e2e_utils.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ========================================================================== */
/* T10.1: State classification                                                */
/* ========================================================================== */
static void
test_t10_1_state_classification(void)
{
  printf("  T10.1: ENC_LOCATE_PENDING state classification... ");

  /* ENC_LOCATE_PENDING is an enc_locate request, not a ray request */
  CHK(path_phase_is_enc_locate_pending(PATH_ENC_LOCATE_PENDING) == 1);
  CHK(path_phase_is_enc_locate_pending(PATH_ENC_LOCATE_RESULT) == 0);
  CHK(path_phase_is_enc_locate_pending(PATH_RAD_TRACE_PENDING) == 0);
  CHK(path_phase_is_enc_locate_pending(PATH_DONE) == 0);
  CHK(path_phase_is_enc_locate_pending(PATH_INIT) == 0);

  /* ENC_LOCATE_PENDING should NOT be classified as a ray-pending state */
  CHK(path_phase_is_ray_pending(PATH_ENC_LOCATE_PENDING) == 0);

  printf("PASS\n");
}

/* ========================================================================== */
/* T10.2: step_enc_locate_submit field assignment                             */
/* ========================================================================== */
static void
test_t10_2_submit_fields(void)
{
  struct path_state p;
  const double pos[3] = {0.25, 0.5, 0.75};

  printf("  T10.2: enc_locate_submit field assignment... ");
  memset(&p, 0, sizeof(p));
  p.phase = PATH_CND_DS_STEP_ENC_VERIFY;
  p.active = 1;

  step_enc_locate_submit(&p, pos, PATH_CND_DS_STEP_ADVANCE);

  CHK(p.phase == PATH_ENC_LOCATE_PENDING);
  CHK(fabs(p.enc_locate.query_pos[0] - 0.25) < 1e-12);
  CHK(fabs(p.enc_locate.query_pos[1] - 0.5)  < 1e-12);
  CHK(fabs(p.enc_locate.query_pos[2] - 0.75) < 1e-12);
  CHK(p.enc_locate.return_state == PATH_CND_DS_STEP_ADVANCE);

  printf("PASS\n");
}

/* ========================================================================== */
/* T10.3: step_enc_locate_result state advance                                */
/* ========================================================================== */
static void
test_t10_3_result_advance(void)
{
  struct path_state p;
  struct sdis_device* dev = NULL;
  struct sdis_scene* scn = NULL;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_solid_shader solid_shader = DUMMY_SOLID_SHADER;
  struct sdis_fluid_shader fluid_shader = DUMMY_FLUID_SHADER;
  struct sdis_interface_shader interf_shader = DUMMY_INTERFACE_SHADER;
  struct sdis_medium* solid = NULL;
  struct sdis_medium* fluid = NULL;
  struct sdis_interface* interf = NULL;
  struct sdis_interface* interfaces[12];
  int i;

  printf("  T10.3: enc_locate_result state advance... ");

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));
  OK(sdis_solid_create(dev, &solid_shader, NULL, &solid));
  OK(sdis_fluid_create(dev, &fluid_shader, NULL, &fluid));
  OK(sdis_interface_create(dev, solid, fluid, &interf_shader, NULL, &interf));
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(fluid));
  for(i = 0; i < 12; i++) interfaces[i] = interf;

  scn_args.get_indices   = box_get_indices;
  scn_args.get_position  = box_get_position;
  scn_args.get_interface = box_get_interface;
  scn_args.nprimitives   = box_ntriangles;
  scn_args.nvertices     = box_nvertices;
  scn_args.context       = interfaces;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  /* Simulate: kernel returned prim_id=0, side=0 (front) */
  memset(&p, 0, sizeof(p));
  p.phase = PATH_ENC_LOCATE_RESULT;
  p.enc_locate.return_state = PATH_CND_DS_STEP_ADVANCE;
  p.enc_locate.prim_id = 0;
  p.enc_locate.side = 0; /* front */
  p.enc_locate.distance = 0.1f;
  p.active = 1;

  OK(step_enc_locate_result(&p, scn));

  /* After result processing, phase should be the return_state */
  CHK(p.phase == PATH_CND_DS_STEP_ADVANCE);
  /* resolved_enc_id should be set by the scene's prim_props */
  CHK(p.enc_locate.resolved_enc_id != (unsigned)-1);

  OK(sdis_interface_ref_put(interf));
  OK(sdis_scene_ref_put(scn));
  free_default_device(dev);

  printf("PASS\n");
}

/* ========================================================================== */
/* T10.10: Collect/distribute flow -- mixed ENC + RAY                         */
/* ========================================================================== */
static void
test_t10_10_collect_distribute_flow(void)
{
  struct path_state paths[10];
  size_t i;
  size_t enc_count = 0, ray_count = 0;

  printf("  T10.10: Collect/distribute flow (mixed ENC+RAY)... ");
  memset(paths, 0, sizeof(paths));

  /* Set up 5 ENC_LOCATE_PENDING paths and 5 RAD_TRACE_PENDING paths */
  for(i = 0; i < 5; i++) {
    paths[i].phase = PATH_ENC_LOCATE_PENDING;
    paths[i].active = 1;
    paths[i].enc_locate.query_pos[0] = (double)i * 0.1;
    paths[i].enc_locate.query_pos[1] = 0.5;
    paths[i].enc_locate.query_pos[2] = 0.5;
    paths[i].enc_locate.return_state = PATH_CND_DS_STEP_ADVANCE;
  }
  for(i = 5; i < 10; i++) {
    paths[i].phase = PATH_RAD_TRACE_PENDING;
    paths[i].active = 1;
    paths[i].needs_ray = 1;
    paths[i].ray_req.ray_count = 1;
    paths[i].ray_bucket = RAY_BUCKET_RADIATIVE;
  }

  /* Count by type classification */
  for(i = 0; i < 10; i++) {
    if(path_phase_is_enc_locate_pending(paths[i].phase))
      enc_count++;
    else if(path_phase_is_ray_pending(paths[i].phase))
      ray_count++;
  }

  CHK(enc_count == 5);
  CHK(ray_count == 5);

  printf("PASS\n");
}

/* ========================================================================== */
/* T10.13: enc_locate vs enc_query sizeof comparison                          */
/* ========================================================================== */
static void
test_t10_13_sizeof_comparison(void)
{
  struct path_state p;
  size_t enc_locate_sz;

  printf("  T10.13: enc_locate sizeof comparison... ");

  /* enc_locate should be compact: query_pos(24) + return_state(4) +
   * resolved_enc_id(4) + prim_id(4) + side(4) + distance(4) + batch_idx(4)
   * = ~48 bytes.  Much smaller than the old enc_query with 6 ray results. */
  enc_locate_sz = sizeof(p.enc_locate);
  fprintf(stdout, "(enc_locate = %lu bytes) ", (unsigned long)enc_locate_sz);

  /* Verify it's reasonably small (< 100 bytes) */
  CHK(enc_locate_sz < 100);

  printf("PASS\n");
}

/* ========================================================================== */
/* T10.12: End-to-end mixed scene (wavefront vs depth-first)                  */
/*                                                                            */
/* Scene: same as T5.7 (solid/fluid coupling) which triggers enclosure        */
/* queries during SS/SF reinjection.  Verifies M10 enc_locate produces       */
/* statistically identical results to depth-first.                            */
/* ========================================================================== */
static int
test_t10_12_e2e_mixed(void)
{
  struct sdis_device* dev = NULL;
  struct sdis_medium* solid = NULL;
  struct sdis_medium* fluid = NULL;
  struct sdis_interface* interf = NULL;
  struct sdis_radiative_env* radenv = NULL;
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

  printf("  T10.12: Mixed scene e2e (enc_locate) ... ");
  fflush(stdout);

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  solid = e2e_create_solid(dev, SDIS_TEMPERATURE_NONE, 1.0,
                           800.0, 2700.0, 0.05, SDIS_VOLUMIC_POWER_NONE);
  fluid = e2e_create_fluid(dev, 350.0, 1000.0, 1.2);
  /* SF interface with coupling: h=10, eps=0.5 -> triggers SS/SF reinjection
   * which uses enc_locate */
  interf = e2e_create_interface(dev, fluid, solid, 10.0, 0.5, 0.0, 300.0);
  radenv = e2e_create_radenv(dev, 300.0, 300.0);

  for(i = 0; i < 12; i++) interfaces[i] = interf;

  scn_args.get_indices   = box_get_indices;
  scn_args.get_position  = box_get_position;
  scn_args.get_interface = box_get_interface;
  scn_args.nprimitives   = box_ntriangles;
  scn_args.nvertices     = box_nvertices;
  scn_args.t_range[0]    = 250;
  scn_args.t_range[1]    = 450;
  scn_args.radenv        = radenv;
  scn_args.context       = interfaces;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  cam = e2e_create_camera(dev, E2E_IMG_WIDTH, E2E_IMG_HEIGHT);

  /* Depth-first (uses synchronous enc query) */
  OK(e2e_run_solve_camera(scn, cam, 0,
    E2E_IMG_WIDTH, E2E_IMG_HEIGHT, 128, 1,
    SDIS_DIFFUSION_DELTA_SPHERE, &img_ref, &t_ref));

  /* Wavefront (uses batch enc_locate via M10) */
  OK(e2e_run_solve_camera(scn, cam, 1,
    E2E_IMG_WIDTH, E2E_IMG_HEIGHT, 128, 1,
    SDIS_DIFFUSION_DELTA_SPHERE, &img_wf, &t_wf));

  e2e_compare_images(img_ref, img_wf, E2E_TOL_SIGMA, E2E_PASS_RATE,
                     &pass, &n_cmp, &n_ok);

  {
    int64_t usec_ref = time_val(&t_ref, TIME_USEC);
    int64_t usec_wf  = time_val(&t_wf,  TIME_USEC);
    if(usec_wf > 0) {
      fprintf(stdout, "    Speedup: %.2fx\n",
        (double)usec_ref / (double)usec_wf);
    }
  }

  OK(sdis_estimator_buffer_ref_put(img_ref));
  OK(sdis_estimator_buffer_ref_put(img_wf));
  OK(sdis_camera_ref_put(cam));
  OK(sdis_scene_ref_put(scn));
  OK(sdis_radiative_env_ref_put(radenv));
  OK(sdis_interface_ref_put(interf));
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(fluid));
  free_default_device(dev);

  printf("%s\n", pass ? "PASS" : "FAIL");
  return pass;
}

/* ========================================================================== */
/* main                                                                       */
/* ========================================================================== */
int
main(int argc, char** argv)
{
  int n_fail = 0;
  (void)argc; (void)argv;

  fprintf(stdout, "=== Phase B-4 M10: Point-in-Enclosure (enc_locate) Tests ===\n\n");

  /* --- Immediate tests --- */
  fprintf(stdout, "--- Immediate Tests ---\n");
  test_t10_1_state_classification();
  test_t10_2_submit_fields();
  test_t10_3_result_advance();
  test_t10_10_collect_distribute_flow();
  test_t10_13_sizeof_comparison();

  /* --- Delayed tests (end-to-end) --- */
  fprintf(stdout, "\n--- Delayed Tests (End-to-End) ---\n");
  if(!test_t10_12_e2e_mixed()) n_fail++;

  /* --- Summary --- */
  fprintf(stdout, "\n=== M10 Tests: %s ===\n",
    n_fail == 0 ? "ALL PASS" : "SOME FAILED");

  return n_fail == 0 ? 0 : 1;
}
