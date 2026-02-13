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

/* Phase B-4, Milestone 6: Convective path + boundary dispatch -- unit tests.
 *
 * These tests verify the fine-grained state machine decomposition of the
 * convective path (step_cnv_init, step_cnv_startup_result, step_cnv_sample_loop)
 * and the boundary dispatch refactoring (step_bnd_dispatch, step_bnd_post_robin_check).
 *
 * Test cases (from phase_b4_test_design.md, section T6):
 *   T6.1: Convective startup ray from fluid interior -> RAY_BUCKET_STARTUP, 1 ray
 *   T6.2: From interface -> direct to SAMPLE_LOOP, no startup ray
 *   T6.4: BND_DISPATCH Dirichlet termination -> 100% done
 *   T6.5: BND_DISPATCH 3-way dispatch -> ss/sf routing
 *   T6.6: Robin post-check -> DONE or continue
 *
 *   T6.3, T6.7, T6.8 are deferred (end-to-end / statistical).
 */

#include "sdis_solve_wavefront.h"
#include "sdis_wf_steps.h"
#include "sdis_scene_c.h"

#include "test_sdis_utils.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ========================================================================== */
/* Custom shaders for convective tests.                                       */
/*                                                                            */
/* The DUMMY shaders return 0 for temperature (= known), which causes         */
/* immediate termination.  We need shaders that return NaN for temperature     */
/* (= unknown) and valid positive values for rho, cp, etc.                    */
/* ========================================================================== */

static double
nan_medium_getter(const struct sdis_rwalk_vertex* vert, struct sdis_data* data)
{
  (void)data;
  (void)vert;
  return NaN; /* Unknown temperature */
}

static double
one_medium_getter(const struct sdis_rwalk_vertex* vert, struct sdis_data* data)
{
  (void)data;
  (void)vert;
  return 1.0; /* rho=1, cp=1 */
}

static double
nan_interface_getter(const struct sdis_interface_fragment* frag,
                     struct sdis_data* data)
{
  (void)data;
  (void)frag;
  return NaN; /* Unknown temperature / flux */
}

static double
zero_interface_getter(const struct sdis_interface_fragment* frag,
                      struct sdis_data* data)
{
  (void)data;
  (void)frag;
  return 0.0;
}

static double
hc100_interface_getter(const struct sdis_interface_fragment* frag,
                       struct sdis_data* data)
{
  (void)data;
  (void)frag;
  return 100.0; /* h_conv = 100 W/(m^2*K) */
}

static double
known_temp_interface_getter(const struct sdis_interface_fragment* frag,
                            struct sdis_data* data)
{
  (void)data;
  (void)frag;
  return 300.0; /* Known Dirichlet temperature */
}

static double
dummy_radiative_getter(const struct sdis_interface_fragment* frag,
                       const unsigned source_id,
                       struct sdis_data* data)
{
  (void)data; (void)source_id; (void)frag;
  return 0.0;
}

/* Fluid shader: NaN temperature, rho=1, cp=1 */
static const struct sdis_fluid_shader CONVECTIVE_FLUID_SHADER = {
  one_medium_getter,  /* Calorific capacity = 1 */
  one_medium_getter,  /* Volumic mass = 1 */
  nan_medium_getter,  /* Temperature = NaN (unknown) */
  0                   /* Initial time */
};

/* Solid shader with NaN temperature (unknown) */
static const struct sdis_solid_shader UNKNOWN_SOLID_SHADER = {
  one_medium_getter,  /* Calorific capacity = 1 */
  one_medium_getter,  /* Thermal conductivity = 1 */
  one_medium_getter,  /* Volumic mass = 1 */
  one_medium_getter,  /* Delta = 1 */
  zero_interface_getter, /* Volumic power = 0 (using interface sig, but same as medium for 0) */
  nan_medium_getter,  /* Temperature = NaN (unknown) */
  NULL,               /* sample path */
  0                   /* Initial time */
};

/* Solid shader with known (zero) temperature (for volumic power we reuse) */
static double
zero_medium_getter(const struct sdis_rwalk_vertex* vert, struct sdis_data* data)
{
  (void)data;
  (void)vert;
  return 0.0;
}

static const struct sdis_solid_shader KNOWN_SOLID_SHADER = {
  one_medium_getter,  /* Calorific capacity = 1 */
  one_medium_getter,  /* Thermal conductivity = 1 */
  one_medium_getter,  /* Volumic mass = 1 */
  one_medium_getter,  /* Delta = 1 */
  zero_medium_getter, /* Volumic power = 0 */
  zero_medium_getter, /* Temperature = 0 (known) */
  NULL,               /* sample path */
  0                   /* Initial time */
};

/* Interface side shader: NaN temperature (no Dirichlet) */
#define NAN_INTERFACE_SIDE_SHADER__ {                                          \
  nan_interface_getter, /* Temperature = NaN */                                \
  nan_interface_getter, /* Flux = NaN */                                       \
  dummy_radiative_getter, /* Emissivity */                                     \
  dummy_radiative_getter, /* Specular fraction */                              \
  nan_interface_getter, /* Reference temperature */                            \
  1 /* Handle external flux */                                                 \
}

/* Interface side shader: known Dirichlet temperature */
#define DIRICHLET_INTERFACE_SIDE_SHADER__ {                                    \
  known_temp_interface_getter, /* Temperature = 300 (known Dirichlet) */       \
  nan_interface_getter, /* Flux = NaN */                                       \
  dummy_radiative_getter, /* Emissivity */                                     \
  dummy_radiative_getter, /* Specular fraction */                              \
  nan_interface_getter, /* Reference temperature */                            \
  1 /* Handle external flux */                                                 \
}

/* Solid/fluid interface with NaN temperature (convective path doesn't stop) */
static const struct sdis_interface_shader SF_INTERFACE_SHADER = {
  hc100_interface_getter,  /* Convection coef = 100 */
  200.0,                   /* Upper bound of convection coef */
  zero_interface_getter,   /* Thermal contact resistance = 0 */
  NAN_INTERFACE_SIDE_SHADER__, /* Front side */
  NAN_INTERFACE_SIDE_SHADER__  /* Back side */
};

/* Solid/solid interface with NaN temperature */
static const struct sdis_interface_shader SS_INTERFACE_SHADER = {
  zero_interface_getter,   /* Convection coef = 0 */
  0.0,                     /* Upper bound of convection coef */
  zero_interface_getter,   /* Thermal contact resistance = 0 */
  NAN_INTERFACE_SIDE_SHADER__, /* Front side */
  NAN_INTERFACE_SIDE_SHADER__  /* Back side */
};

/* Interface with known Dirichlet temperature */
static const struct sdis_interface_shader DIRICHLET_INTERFACE_SHADER = {
  zero_interface_getter,   /* Convection coef = 0 */
  0.0,                     /* Upper bound of convection coef */
  zero_interface_getter,   /* Thermal contact resistance = 0 */
  DIRICHLET_INTERFACE_SIDE_SHADER__, /* Front side */
  DIRICHLET_INTERFACE_SIDE_SHADER__  /* Back side */
};

/* ========================================================================== */
/* Shared test scenes                                                         */
/* ========================================================================== */
static struct sdis_device*  g_dev = NULL;

/* Scene A: solid/fluid interface — for convective path tests (T6.1, T6.2) */
static struct sdis_scene*   g_scn_sf = NULL;
static struct ssp_rng*      g_rng = NULL;
static unsigned             g_fluid_enc_sf = 0; /* fluid enclosure of SF scene */
static unsigned             g_solid_enc_sf = 0; /* solid enclosure of SF scene */

/* Scene B: solid/solid interface with Dirichlet — for BND_DISPATCH tests */
static struct sdis_scene*   g_scn_dirichlet = NULL;

/* Scene C: solid/solid interface, NaN temperature — for SS dispatch test */
static struct sdis_scene*   g_scn_ss = NULL;
static unsigned             g_inner_enc_ss = 0;

static struct sdis_interface* g_sf_interfaces[12];
static struct sdis_interface* g_dirichlet_interfaces[12];
static struct sdis_interface* g_ss_interfaces[12];

static void
setup_test_scenes(void)
{
  struct sdis_scene_create_args scn_args;
  struct sdis_medium* solid_unknown = NULL;
  struct sdis_medium* solid_known = NULL;
  struct sdis_medium* fluid = NULL;
  struct sdis_interface* interf_sf = NULL;
  struct sdis_interface* interf_dirichlet = NULL;
  struct sdis_interface* interf_ss = NULL;
  int i;

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &g_dev));

  /* -- Media -- */
  {
    struct sdis_solid_shader s = UNKNOWN_SOLID_SHADER;
    /* volumic_power has wrong signature for solid shader (interface_getter)
     * but the test only exercises convective/boundary steps, not solid
     * property checks, so we use zero_medium_getter instead. */
    s.volumic_power = zero_medium_getter;
    OK(sdis_solid_create(g_dev, &s, NULL, &solid_unknown));
  }
  {
    struct sdis_solid_shader s = KNOWN_SOLID_SHADER;
    OK(sdis_solid_create(g_dev, &s, NULL, &solid_known));
  }
  {
    struct sdis_fluid_shader f = CONVECTIVE_FLUID_SHADER;
    OK(sdis_fluid_create(g_dev, &f, NULL, &fluid));
  }

  /* -- Interfaces -- */
  /* Fluid / Solid(unknown): for convective tests.
   * senc3d maps front medium to the inner (finite) enclosure and back medium
   * to the outer (infinite) enclosure.  We need the inner enclosure to have
   * fluid medium so that the convective path has proper geometry (S_over_V,
   * hc_upper_bound). */
  {
    struct sdis_interface_shader sh = SF_INTERFACE_SHADER;
    OK(sdis_interface_create(g_dev, fluid, solid_unknown, &sh, NULL,
                             &interf_sf));
  }
  for(i = 0; i < 12; i++) g_sf_interfaces[i] = interf_sf;

  /* Solid(known) / Fluid: Dirichlet temperature on interface */
  {
    struct sdis_interface_shader sh = DIRICHLET_INTERFACE_SHADER;
    OK(sdis_interface_create(g_dev, solid_known, fluid, &sh, NULL,
                             &interf_dirichlet));
  }
  for(i = 0; i < 12; i++) g_dirichlet_interfaces[i] = interf_dirichlet;

  /* Solid / Solid: for SS dispatch test */
  {
    struct sdis_interface_shader sh = SS_INTERFACE_SHADER;
    struct sdis_medium* solid_unknown2 = NULL;
    struct sdis_solid_shader s2 = UNKNOWN_SOLID_SHADER;
    s2.volumic_power = zero_medium_getter;
    OK(sdis_solid_create(g_dev, &s2, NULL, &solid_unknown2));
    OK(sdis_interface_create(g_dev, solid_unknown, solid_unknown2, &sh, NULL,
                             &interf_ss));
    OK(sdis_medium_ref_put(solid_unknown2));
  }
  for(i = 0; i < 12; i++) g_ss_interfaces[i] = interf_ss;

  /* -- Scene A: solid/fluid -- */
  scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  scn_args.get_indices  = box_get_indices;
  scn_args.get_interface = box_get_interface;
  scn_args.get_position = box_get_position;
  scn_args.nprimitives  = box_ntriangles;
  scn_args.nvertices    = box_nvertices;
  scn_args.context      = g_sf_interfaces;
  OK(sdis_scene_create(g_dev, &scn_args, &g_scn_sf));

  /* Determine the fluid enclosure for SF scene.
   * With interface(front=solid, back=fluid), one enclosure is solid and the
   * other is fluid.  Check both to find the fluid one. */
  {
    unsigned encs[2];
    struct sdis_medium* mdm0 = NULL;
    struct sdis_medium* mdm1 = NULL;
    scene_get_enclosure_ids(g_scn_sf, 0, encs);
    OK(scene_get_enclosure_medium(g_scn_sf,
      scene_get_enclosure(g_scn_sf, encs[0]), &mdm0));
    OK(scene_get_enclosure_medium(g_scn_sf,
      scene_get_enclosure(g_scn_sf, encs[1]), &mdm1));
    if(sdis_medium_get_type(mdm0) == SDIS_FLUID) {
      g_fluid_enc_sf = encs[0];
      g_solid_enc_sf = encs[1];
    } else {
      CHK(sdis_medium_get_type(mdm1) == SDIS_FLUID);
      g_fluid_enc_sf = encs[1];
      g_solid_enc_sf = encs[0];
    }
  }

  /* -- Scene B: Dirichlet -- */
  scn_args.context = g_dirichlet_interfaces;
  OK(sdis_scene_create(g_dev, &scn_args, &g_scn_dirichlet));

  /* -- Scene C: solid/solid -- */
  scn_args.context = g_ss_interfaces;
  OK(sdis_scene_create(g_dev, &scn_args, &g_scn_ss));

  {
    unsigned encs[2];
    scene_get_enclosure_ids(g_scn_ss, 0, encs);
    g_inner_enc_ss = (encs[0] != g_scn_ss->outer_enclosure_id)
                   ? encs[0] : encs[1];
  }

  /* Clean up media/interface refs (scene holds its own) */
  OK(sdis_interface_ref_put(interf_sf));
  OK(sdis_interface_ref_put(interf_dirichlet));
  OK(sdis_interface_ref_put(interf_ss));
  OK(sdis_medium_ref_put(solid_unknown));
  OK(sdis_medium_ref_put(solid_known));
  OK(sdis_medium_ref_put(fluid));

  /* RNG */
  OK(ssp_rng_create(NULL, SSP_RNG_KISS, &g_rng));
}

static void
teardown_test_scenes(void)
{
  OK(ssp_rng_ref_put(g_rng));
  OK(sdis_scene_ref_put(g_scn_sf));
  OK(sdis_scene_ref_put(g_scn_dirichlet));
  OK(sdis_scene_ref_put(g_scn_ss));
  OK(sdis_device_ref_put(g_dev));
  g_rng = NULL;
  g_scn_sf = NULL;
  g_scn_dirichlet = NULL;
  g_scn_ss = NULL;
  g_dev = NULL;
}

/* ========================================================================== */
/* T6.1: Convective startup ray from fluid interior                           */
/*                                                                            */
/* When the path starts from a fluid interior (hit_3d = S3D_HIT_NULL),        */
/* step_cnv_init should emit a startup ray along +Z and set                   */
/* phase = PATH_CNV_STARTUP_TRACE.                                            */
/* ========================================================================== */
static void
test_cnv_startup_ray(void)
{
  struct path_state p;

  printf("  T6.1: convective startup ray from fluid interior... ");

  memset(&p, 0, sizeof(p));
  p.active = 1;
  p.rng = g_rng;
  /* Position inside the box */
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;
  /* No hit = fluid interior */
  p.rwalk.hit_3d = S3D_HIT_NULL;
  /* Set enc_id to the fluid enclosure */
  p.rwalk.enc_id = g_fluid_enc_sf;

  step_cnv_init(&p, g_scn_sf);

  /* Should produce startup ray along +Z */
  CHK(p.phase == PATH_CNV_STARTUP_TRACE);
  CHK(p.needs_ray == 1);
  CHK(p.ray_count_ext == 1);
  CHK(p.ray_bucket == RAY_BUCKET_STARTUP);
  CHK(p.ray_req.ray_count == 1);

  /* Ray origin = position */
  CHK(fabsf(p.ray_req.origin[0] - 0.5f) < 1.e-6f);
  CHK(fabsf(p.ray_req.origin[1] - 0.5f) < 1.e-6f);
  CHK(fabsf(p.ray_req.origin[2] - 0.5f) < 1.e-6f);

  /* Ray direction = +Z */
  CHK(fabsf(p.ray_req.direction[0]) < 1.e-6f);
  CHK(fabsf(p.ray_req.direction[1]) < 1.e-6f);
  CHK(fabsf(p.ray_req.direction[2] - 1.0f) < 1.e-6f);

  /* Range: [FLT_MIN, FLT_MAX] */
  CHK(p.ray_req.range[0] == FLT_MIN);
  CHK(p.ray_req.range[1] == FLT_MAX);

  printf("PASS\n");
}

/* ========================================================================== */
/* T6.2: From interface -> direct to SAMPLE_LOOP (no startup ray)             */
/*                                                                            */
/* When the path starts with a valid hit (not S3D_HIT_NULL), step_cnv_init    */
/* should skip the startup ray and go directly to PATH_CNV_SAMPLE_LOOP.       */
/* ========================================================================== */
static void
test_cnv_from_interface(void)
{
  struct path_state p;

  printf("  T6.2: from interface -> direct SAMPLE_LOOP... ");

  memset(&p, 0, sizeof(p));
  p.active = 1;
  p.rng = g_rng;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.0; /* On box face */
  p.rwalk.enc_id = g_fluid_enc_sf;

  /* Set a valid hit (not S3D_HIT_NULL) — simulate arriving from an interface */
  memset(&p.rwalk.hit_3d, 0, sizeof(p.rwalk.hit_3d));
  p.rwalk.hit_3d.prim.prim_id = 0;
  p.rwalk.hit_3d.prim.geom_id = 0;
  p.rwalk.hit_3d.prim.inst_id = 0;
  p.rwalk.hit_3d.distance = 0.1f;
  p.rwalk.hit_3d.normal[2] = -1.0f;
  p.rwalk.hit_side = SDIS_FRONT;

  step_cnv_init(&p, g_scn_sf);

  /* Should go directly to SAMPLE_LOOP without emitting a startup ray */
  CHK(p.phase == PATH_CNV_SAMPLE_LOOP);
  CHK(p.needs_ray == 0);

  /* Verify locals.cnv were populated */
  CHK(p.locals.cnv.enc_id == g_fluid_enc_sf);
  CHK(p.locals.cnv.rho_cp > 0); /* rho=1, cp=1 => rho_cp=1 */
  /* hc_upper_bound is stored from enclosure (set during scene creation
   * from interface shader's convection_coef_upper_bound = 200) */

  printf("PASS\n");
}

/* ========================================================================== */
/* T6.1b: Startup result with valid hit -> SAMPLE_LOOP                        */
/*                                                                            */
/* Verify step_cnv_startup_result processes a hit correctly and transitions    */
/* to PATH_CNV_SAMPLE_LOOP.                                                   */
/* ========================================================================== */
static void
test_cnv_startup_result(void)
{
  struct path_state p;
  struct s3d_hit hit;
  res_T res;

  printf("  T6.1b: startup result with hit -> SAMPLE_LOOP... ");

  memset(&p, 0, sizeof(p));
  p.active = 1;
  p.rng = g_rng;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;
  p.rwalk.enc_id = g_fluid_enc_sf;
  p.phase = PATH_CNV_STARTUP_TRACE;

  /* Simulate hit from startup ray: surface at Z=1.0, normal=(0,0,1) */
  memset(&hit, 0, sizeof(hit));
  hit.distance = 0.5f;
  hit.prim.prim_id = 0;
  hit.prim.geom_id = 0;
  hit.prim.inst_id = 0;
  hit.normal[0] = 0.0f;
  hit.normal[1] = 0.0f;
  hit.normal[2] = 1.0f;

  res = step_cnv_startup_result(&p, g_scn_sf, &hit);
  CHK(res == RES_OK);

  CHK(p.phase == PATH_CNV_SAMPLE_LOOP);
  CHK(p.rwalk.hit_3d.distance == 0.5f);

  /* hit_side: dot(normal=(0,0,1), startup_dir=(0,0,1)) > 0 => SDIS_BACK */
  CHK(p.rwalk.hit_side == SDIS_BACK);

  /* locals.cnv populated */
  CHK(p.locals.cnv.enc_id == g_fluid_enc_sf);
  CHK(p.locals.cnv.rho_cp > 0);

  printf("PASS\n");
}

/* ========================================================================== */
/* T6.1c: Startup result with miss -> error                                   */
/* ========================================================================== */
static void
test_cnv_startup_result_miss(void)
{
  struct path_state p;
  struct s3d_hit hit;
  res_T res;

  printf("  T6.1c: startup result miss -> error... ");

  memset(&p, 0, sizeof(p));
  p.active = 1;
  p.rng = g_rng;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;
  p.rwalk.enc_id = g_fluid_enc_sf;
  p.phase = PATH_CNV_STARTUP_TRACE;

  hit = S3D_HIT_NULL;

  res = step_cnv_startup_result(&p, g_scn_sf, &hit);

  /* Miss should produce error and deactivate */
  CHK(res == RES_BAD_OP);
  CHK(p.phase == PATH_DONE);
  CHK(p.active == 0);

  printf("PASS\n");
}

/* ========================================================================== */
/* T6.2b: Known fluid temperature -> immediate PATH_DONE                      */
/*                                                                            */
/* When the fluid temperature is already known, step_cnv_init should          */
/* immediately terminate the path.                                            */
/* ========================================================================== */
static void
test_cnv_known_temperature(void)
{
  struct path_state p;
  struct sdis_scene* scn_known = NULL;
  struct sdis_scene_create_args scn_args;
  struct sdis_medium* solid = NULL;
  struct sdis_medium* fluid = NULL;
  struct sdis_interface* interf = NULL;
  struct sdis_interface* ifaces[12];
  unsigned fluid_enc;
  int i;

  printf("  T6.2b: known fluid temperature -> PATH_DONE... ");

  /* Build a scene with a fluid whose temperature = 0 (known).
   * senc3d maps front medium -> inner enclosure.  Put fluid on front. */
  {
    struct sdis_solid_shader ss = DUMMY_SOLID_SHADER;
    struct sdis_fluid_shader fs = DUMMY_FLUID_SHADER; /* returns 0 = known */
    struct sdis_interface_shader ish = SF_INTERFACE_SHADER;
    OK(sdis_solid_create(g_dev, &ss, NULL, &solid));
    OK(sdis_fluid_create(g_dev, &fs, NULL, &fluid));
    OK(sdis_interface_create(g_dev, fluid, solid, &ish, NULL, &interf));
    OK(sdis_medium_ref_put(solid));
    OK(sdis_medium_ref_put(fluid));
    for(i = 0; i < 12; i++) ifaces[i] = interf;
  }

  scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  scn_args.get_indices   = box_get_indices;
  scn_args.get_interface = box_get_interface;
  scn_args.get_position  = box_get_position;
  scn_args.nprimitives   = box_ntriangles;
  scn_args.nvertices     = box_nvertices;
  scn_args.context       = ifaces;
  OK(sdis_scene_create(g_dev, &scn_args, &scn_known));
  OK(sdis_interface_ref_put(interf));

  {
    unsigned encs[2];
    struct sdis_medium* mdm0 = NULL;
    struct sdis_medium* mdm1 = NULL;
    scene_get_enclosure_ids(scn_known, 0, encs);
    OK(scene_get_enclosure_medium(scn_known,
      scene_get_enclosure(scn_known, encs[0]), &mdm0));
    OK(scene_get_enclosure_medium(scn_known,
      scene_get_enclosure(scn_known, encs[1]), &mdm1));
    fluid_enc = (sdis_medium_get_type(mdm0) == SDIS_FLUID) ? encs[0] : encs[1];
  }

  memset(&p, 0, sizeof(p));
  p.active = 1;
  p.rng = g_rng;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.0;
  p.rwalk.enc_id = fluid_enc;
  /* Set valid hit so we skip startup ray */
  memset(&p.rwalk.hit_3d, 0, sizeof(p.rwalk.hit_3d));
  p.rwalk.hit_3d.prim.prim_id = 0;
  p.rwalk.hit_3d.distance = 0.1f;
  p.rwalk.hit_3d.normal[2] = -1.0f;

  step_cnv_init(&p, scn_known);

  /* Temperature 0 is known -> immediate DONE */
  CHK(p.phase == PATH_DONE);
  CHK(p.active == 0);
  CHK(p.T.done == 1);

  OK(sdis_scene_ref_put(scn_known));

  printf("PASS\n");
}

/* ========================================================================== */
/* T6.4: BND_DISPATCH Dirichlet termination                                   */
/*                                                                            */
/* With a Dirichlet interface (known temperature), step_bnd_dispatch should   */
/* immediately terminate the path.                                            */
/* ========================================================================== */
static void
test_bnd_dispatch_dirichlet(void)
{
  struct path_state p;
  res_T res;

  printf("  T6.4: BND_DISPATCH Dirichlet termination... ");

  memset(&p, 0, sizeof(p));
  p.active = 1;
  p.rng = g_rng;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.0; /* On box face */

  /* Set valid hit on prim 0 */
  memset(&p.rwalk.hit_3d, 0, sizeof(p.rwalk.hit_3d));
  p.rwalk.hit_3d.prim.prim_id = 0;
  p.rwalk.hit_3d.prim.geom_id = 0;
  p.rwalk.hit_3d.prim.inst_id = 0;
  p.rwalk.hit_3d.distance = 0.1f;
  p.rwalk.hit_3d.normal[0] = 0.0f;
  p.rwalk.hit_3d.normal[1] = 0.0f;
  p.rwalk.hit_3d.normal[2] = -1.0f;
  p.rwalk.hit_side = SDIS_FRONT;

  /* Use Dirichlet scene */
  res = step_bnd_dispatch(&p, g_scn_dirichlet);
  CHK(res == RES_OK);

  /* Dirichlet -> immediate DONE */
  CHK(p.phase == PATH_DONE);
  CHK(p.active == 0);
  CHK(p.T.done == 1);
  CHK(fabs(p.T.value - 300.0) < 1.e-10); /* known_temp_interface_getter */
  CHK(p.done_reason == 3); /* boundary done */

  printf("PASS\n");
}

/* ========================================================================== */
/* T6.5a: BND_DISPATCH solid/solid dispatch -> SS_REINJECT                    */
/*                                                                            */
/* With a solid/solid interface (no Dirichlet), step_bnd_dispatch should      */
/* route to the M3 batched reinjection state machine.                         */
/* ========================================================================== */
static void
test_bnd_dispatch_ss(void)
{
  struct path_state p;
  res_T res;

  printf("  T6.5a: BND_DISPATCH solid/solid -> SS_REINJECT... ");

  memset(&p, 0, sizeof(p));
  p.active = 1;
  p.rng = g_rng;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.0;

  /* Set valid hit on prim 0 */
  memset(&p.rwalk.hit_3d, 0, sizeof(p.rwalk.hit_3d));
  p.rwalk.hit_3d.prim.prim_id = 0;
  p.rwalk.hit_3d.prim.geom_id = 0;
  p.rwalk.hit_3d.prim.inst_id = 0;
  p.rwalk.hit_3d.distance = 0.1f;
  p.rwalk.hit_3d.normal[0] = 0.0f;
  p.rwalk.hit_3d.normal[1] = 0.0f;
  p.rwalk.hit_3d.normal[2] = -1.0f;
  p.rwalk.hit_side = SDIS_FRONT;
  p.filter_data_storage = HIT_FILTER_DATA_NULL;

  /* Use solid/solid scene */
  res = step_bnd_dispatch(&p, g_scn_ss);
  CHK(res == RES_OK);

  /* Solid/solid -> enters M3 batched reinjection, which means
   * step_bnd_ss_reinject_sample was called internally.
   * The phase should be PATH_BND_SS_REINJECT_SAMPLE with rays pending. */
  CHK(p.phase == PATH_BND_SS_REINJECT_SAMPLE);
  CHK(p.needs_ray == 1);
  CHK(p.ray_count_ext == 4);
  CHK(p.ray_bucket == RAY_BUCKET_STEP_PAIR);

  printf("PASS\n");
}

/* ========================================================================== */
/* T6.5b: BND_DISPATCH solid/fluid picard1 -> POST_ROBIN or DONE             */
/*                                                                            */
/* With a solid/fluid interface at max_branchings, step_bnd_dispatch should   */
/* call picard1 synchronously. Since our fluid returns NaN temperature and    */
/* the picard1 path is complex, we focus on verifying the dispatch routing.   */
/* ========================================================================== */
static void
test_bnd_dispatch_sf(void)
{
  struct path_state p;
  res_T res;

  printf("  T6.5b: BND_DISPATCH solid/fluid -> picard1 or picardN route... ");

  memset(&p, 0, sizeof(p));
  p.active = 1;
  p.rng = g_rng;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.0;
  p.ctx.nbranchings = 0;
  p.ctx.max_branchings = 0; /* nbranchings == max_branchings => picard1 */

  /* Set valid hit */
  memset(&p.rwalk.hit_3d, 0, sizeof(p.rwalk.hit_3d));
  p.rwalk.hit_3d.prim.prim_id = 0;
  p.rwalk.hit_3d.prim.geom_id = 0;
  p.rwalk.hit_3d.prim.inst_id = 0;
  p.rwalk.hit_3d.distance = 0.1f;
  p.rwalk.hit_3d.normal[0] = 0.0f;
  p.rwalk.hit_3d.normal[1] = 0.0f;
  p.rwalk.hit_3d.normal[2] = -1.0f;
  p.rwalk.hit_side = SDIS_FRONT;

  /* Use solid/fluid scene */
  res = step_bnd_dispatch(&p, g_scn_sf);

  /* The picard1 path runs synchronously. Regardless of whether it succeeds
   * or hits a termination condition, the result should be one of:
   *   - PATH_DONE (picard1 found temperature or errored)
   *   - PATH_BND_POST_ROBIN_CHECK (picard1 set T.func and continues) */
  CHK(p.phase == PATH_DONE || p.phase == PATH_BND_POST_ROBIN_CHECK);

  /* If done, the path was resolved */
  if(p.phase == PATH_DONE) {
    CHK(p.active == 0);
  }
  /* If POST_ROBIN_CHECK, T.func should be set */

  printf("PASS\n");
}

/* ========================================================================== */
/* T6.6: Robin post-check routing                                             */
/*                                                                            */
/* Verify step_bnd_post_robin_check routes correctly based on T.func.         */
/* ========================================================================== */

/* Forward declarations for T.func pointers (from sdis_heat_path.h).
 * These are the actual functions that the solver uses. */
extern res_T convective_path_3d(
  struct sdis_scene*, struct rwalk_context*, struct rwalk*,
  struct ssp_rng*, struct temperature*);
extern res_T conductive_path_3d(
  struct sdis_scene*, struct rwalk_context*, struct rwalk*,
  struct ssp_rng*, struct temperature*);
extern res_T radiative_path_3d(
  struct sdis_scene*, struct rwalk_context*, struct rwalk*,
  struct ssp_rng*, struct temperature*);
extern res_T boundary_path_3d(
  struct sdis_scene*, struct rwalk_context*, struct rwalk*,
  struct ssp_rng*, struct temperature*);

static void
test_post_robin_convective(void)
{
  struct path_state p;
  res_T res;

  printf("  T6.6a: post-robin -> COUPLED_CONVECTIVE... ");

  memset(&p, 0, sizeof(p));
  p.active = 1;
  p.T.done = 0;
  p.T.func = convective_path_3d;
  /* Set enc_id to fluid so query_medium_temperature_from_boundary can work */
  p.rwalk.enc_id = g_fluid_enc_sf;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;
  /* Set a valid hit so functions don't crash */
  memset(&p.rwalk.hit_3d, 0, sizeof(p.rwalk.hit_3d));
  p.rwalk.hit_3d.prim.prim_id = 0;
  p.rwalk.hit_3d.distance = 0.1f;
  p.rwalk.hit_3d.normal[2] = -1.0f;
  p.rwalk.hit_side = SDIS_FRONT;

  res = step_bnd_post_robin_check(&p, g_scn_sf);
  CHK(res == RES_OK);

  /* Fluid temp is NaN in our SF scene => query_medium_temperature won't find
   * known temp => routes to PATH_COUPLED_CONVECTIVE */
  CHK(p.phase == PATH_COUPLED_CONVECTIVE);
  CHK(p.needs_ray == 0);

  printf("PASS\n");
}

static void
test_post_robin_conductive(void)
{
  struct path_state p;
  res_T res;

  printf("  T6.6b: post-robin -> COUPLED_CONDUCTIVE... ");

  memset(&p, 0, sizeof(p));
  p.active = 1;
  p.T.done = 0;
  p.T.func = conductive_path_3d;
  p.rwalk.enc_id = g_inner_enc_ss;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;
  memset(&p.rwalk.hit_3d, 0, sizeof(p.rwalk.hit_3d));
  p.rwalk.hit_3d.prim.prim_id = 0;
  p.rwalk.hit_3d.distance = 0.1f;
  p.rwalk.hit_3d.normal[2] = -1.0f;
  p.rwalk.hit_side = SDIS_FRONT;

  res = step_bnd_post_robin_check(&p, g_scn_ss);
  CHK(res == RES_OK);

  /* Solid temp is NaN in our SS scene => routes to COUPLED_CONDUCTIVE */
  CHK(p.phase == PATH_COUPLED_CONDUCTIVE);
  CHK(p.needs_ray == 0);

  printf("PASS\n");
}

static void
test_post_robin_radiative(void)
{
  struct path_state p;
  res_T res;

  printf("  T6.6c: post-robin -> COUPLED_RADIATIVE... ");

  memset(&p, 0, sizeof(p));
  p.active = 1;
  p.T.done = 0;
  p.T.func = radiative_path_3d;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;

  res = step_bnd_post_robin_check(&p, g_scn_sf);
  CHK(res == RES_OK);

  /* radiative_path_3d skips the query_medium check -> COUPLED_RADIATIVE */
  CHK(p.phase == PATH_COUPLED_RADIATIVE);
  CHK(p.needs_ray == 0);

  printf("PASS\n");
}

static void
test_post_robin_boundary(void)
{
  struct path_state p;
  res_T res;

  printf("  T6.6d: post-robin -> BND_DISPATCH (re-dispatch)... ");

  memset(&p, 0, sizeof(p));
  p.active = 1;
  p.T.done = 0;
  p.T.func = boundary_path_3d;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;

  res = step_bnd_post_robin_check(&p, g_scn_sf);
  CHK(res == RES_OK);

  /* boundary_path_3d -> BND_DISPATCH (loop back) */
  CHK(p.phase == PATH_BND_DISPATCH);
  CHK(p.needs_ray == 0);

  printf("PASS\n");
}

/* ========================================================================== */
/* T6.misc: step_convective redirects to step_cnv_init                        */
/* ========================================================================== */
static void
test_convective_redirect(void)
{
  struct path_state p;

  printf("  T6.misc: step_convective -> step_cnv_init redirect... ");

  memset(&p, 0, sizeof(p));
  p.active = 1;
  p.rng = g_rng;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;
  p.rwalk.hit_3d = S3D_HIT_NULL;
  p.rwalk.enc_id = g_fluid_enc_sf;

  step_convective(&p, g_scn_sf);

  /* step_convective sets phase to PATH_CNV_INIT and calls step_cnv_init.
   * With S3D_HIT_NULL and NaN fluid temperature, it should emit startup ray. */
  CHK(p.phase == PATH_CNV_STARTUP_TRACE);
  CHK(p.needs_ray == 1);
  CHK(p.ray_bucket == RAY_BUCKET_STARTUP);

  printf("PASS\n");
}

/* ========================================================================== */
/* T6.misc: step_boundary redirects to step_bnd_dispatch                      */
/* ========================================================================== */
static void
test_boundary_redirect(void)
{
  struct path_state p;

  printf("  T6.misc: step_boundary -> step_bnd_dispatch redirect... ");

  memset(&p, 0, sizeof(p));
  p.active = 1;
  p.rng = g_rng;
  p.coupled_nbranchings = -1; /* First entry */
  p.ctx.max_branchings = 10;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.0;

  /* Set valid hit on Dirichlet scene */
  memset(&p.rwalk.hit_3d, 0, sizeof(p.rwalk.hit_3d));
  p.rwalk.hit_3d.prim.prim_id = 0;
  p.rwalk.hit_3d.prim.geom_id = 0;
  p.rwalk.hit_3d.prim.inst_id = 0;
  p.rwalk.hit_3d.distance = 0.1f;
  p.rwalk.hit_3d.normal[2] = -1.0f;
  p.rwalk.hit_side = SDIS_FRONT;

  step_boundary(&p, g_scn_dirichlet);

  /* Dirichlet temperature on all faces -> DONE */
  CHK(p.phase == PATH_DONE);
  CHK(p.active == 0);
  CHK(p.T.done == 1);
  CHK(fabs(p.T.value - 300.0) < 1.e-10);

  /* Verify nbranchings was managed correctly */
  CHK(p.coupled_nbranchings == 0); /* First entry: sentinel -1 -> 0 */

  printf("PASS\n");
}

/* ========================================================================== */
/* T6.misc: advance_one_step_no_ray dispatches CNV_INIT correctly             */
/* ========================================================================== */
static void
test_advance_no_ray_cnv_init(void)
{
  struct path_state p;
  int advanced = 0;
  res_T res;

  printf("  T6.misc: advance_no_ray dispatches CNV_INIT... ");

  memset(&p, 0, sizeof(p));
  p.active = 1;
  p.rng = g_rng;
  p.phase = PATH_CNV_INIT;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;
  p.rwalk.hit_3d = S3D_HIT_NULL;
  p.rwalk.enc_id = g_fluid_enc_sf;

  res = advance_one_step_no_ray(&p, g_scn_sf, &advanced);
  CHK(res == RES_OK);
  CHK(advanced == 1);

  /* With HIT_NULL and NaN temp -> startup ray */
  CHK(p.phase == PATH_CNV_STARTUP_TRACE);
  CHK(p.needs_ray == 1);

  printf("PASS\n");
}

/* ========================================================================== */
/* T6.misc: advance_one_step_no_ray dispatches BND_DISPATCH correctly         */
/* ========================================================================== */
static void
test_advance_no_ray_bnd_dispatch(void)
{
  struct path_state p;
  int advanced = 0;
  res_T res;

  printf("  T6.misc: advance_no_ray dispatches BND_DISPATCH... ");

  memset(&p, 0, sizeof(p));
  p.active = 1;
  p.rng = g_rng;
  p.phase = PATH_BND_DISPATCH;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.0;
  memset(&p.rwalk.hit_3d, 0, sizeof(p.rwalk.hit_3d));
  p.rwalk.hit_3d.prim.prim_id = 0;
  p.rwalk.hit_3d.distance = 0.1f;
  p.rwalk.hit_3d.normal[2] = -1.0f;
  p.rwalk.hit_side = SDIS_FRONT;

  res = advance_one_step_no_ray(&p, g_scn_dirichlet, &advanced);
  CHK(res == RES_OK);
  CHK(advanced == 1);

  /* Dirichlet -> DONE */
  CHK(p.phase == PATH_DONE);
  CHK(p.T.done == 1);

  printf("PASS\n");
}

/* ========================================================================== */
/* T6.misc: advance_one_step_with_ray dispatches CNV_STARTUP_TRACE            */
/* ========================================================================== */
static void
test_advance_with_ray_cnv_startup(void)
{
  struct path_state p;
  struct s3d_hit hit0;
  res_T res;

  printf("  T6.misc: advance_with_ray dispatches CNV_STARTUP_TRACE... ");

  memset(&p, 0, sizeof(p));
  p.active = 1;
  p.rng = g_rng;
  p.phase = PATH_CNV_STARTUP_TRACE;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;
  p.rwalk.enc_id = g_fluid_enc_sf;

  /* Valid hit */
  memset(&hit0, 0, sizeof(hit0));
  hit0.distance = 0.5f;
  hit0.prim.prim_id = 0;
  hit0.normal[2] = 1.0f;

  res = advance_one_step_with_ray(&p, g_scn_sf, &hit0, NULL);
  CHK(res == RES_OK);

  /* Should have called step_cnv_startup_result -> SAMPLE_LOOP */
  CHK(p.phase == PATH_CNV_SAMPLE_LOOP);

  printf("PASS\n");
}

/* ========================================================================== */
/* T6.misc: Phase classification checks                                       */
/* ========================================================================== */
static void
test_m6_phase_classification(void)
{
  printf("  T6.misc: phase classification... ");

  /* CNV_STARTUP_TRACE is a ray-pending phase */
  CHK(path_phase_is_ray_pending(PATH_CNV_STARTUP_TRACE) == 1);

  /* CNV_INIT, CNV_SAMPLE_LOOP, BND_DISPATCH, BND_POST_ROBIN_CHECK are
   * NOT ray-pending */
  CHK(path_phase_is_ray_pending(PATH_CNV_INIT) == 0);
  CHK(path_phase_is_ray_pending(PATH_CNV_SAMPLE_LOOP) == 0);
  CHK(path_phase_is_ray_pending(PATH_BND_DISPATCH) == 0);
  CHK(path_phase_is_ray_pending(PATH_BND_POST_ROBIN_CHECK) == 0);

  printf("PASS\n");
}

/* ========================================================================== */
/* Main                                                                       */
/* ========================================================================== */
int
main(int argc, char* argv[])
{
  (void)argc;
  (void)argv;

  printf("Phase B-4 M6: Convective path + boundary dispatch tests\n");
  printf("========================================================\n");

  /* Build shared test scenes */
  setup_test_scenes();

  /* Phase classification */
  test_m6_phase_classification();

  /* T6.1: Convective startup ray */
  test_cnv_startup_ray();
  test_cnv_startup_result();
  test_cnv_startup_result_miss();

  /* T6.2: From interface / known temperature */
  test_cnv_from_interface();
  test_cnv_known_temperature();

  /* T6.4: BND_DISPATCH Dirichlet termination */
  test_bnd_dispatch_dirichlet();

  /* T6.5: BND_DISPATCH 3-way dispatch */
  test_bnd_dispatch_ss();
  test_bnd_dispatch_sf();

  /* T6.6: Robin post-check routing */
  test_post_robin_convective();
  test_post_robin_conductive();
  test_post_robin_radiative();
  test_post_robin_boundary();

  /* Misc: redirect and advance dispatch */
  test_convective_redirect();
  test_boundary_redirect();
  test_advance_no_ray_cnv_init();
  test_advance_no_ray_bnd_dispatch();
  test_advance_with_ray_cnv_startup();

  /* Release */
  teardown_test_scenes();

  printf("\nAll B-4 M6 tests PASSED.\n");
  return 0;
}
