/* WF-F2: External flux with diffuse radiance (wavefront probe).
 *
 * Scene: small solid sphere (R=0.01 m) illuminated by a distant spherical
 *        source with an additional diffuse radiance component.
 *
 *   Sphere: solid, lambda=1, rho=1, cp=1, delta=0.002, eps=1
 *   Source: P=1e5 W, R_src=0.3 m, d=10 m, position=(10,0,0)
 *   Radenv: T_rad=300K, T_ref=300K
 *   Interface: eps=1 on front (fluid side), Tref=320K
 *
 * Analytic (surface temperature of uniformly irradiated sphere):
 *   q   = P / (4*pi*d^2) ≈ 79.577 W/m^2
 *   Ts  = [ (q/4 + Ldiff*pi) / sigma + T_rad^4 ]^0.25
 *
 * With Ldiff=50 W/m^2/sr:
 *   Ts  ≈ [ (19.894 + 157.08) / 5.6696e-8 + 300^4 ]^0.25
 *
 * Probe at sphere centre (0,0,0).  For a small sphere with lambda=1,
 * the temperature gradient is negligible → probe ≈ surface T.
 *
 * Reference CPU test: test_sdis_external_flux_with_diffuse_radiance.c
 */

#include "sdis.h"
#include "test_sdis_utils.h"
#include "test_sdis_wf_p0_utils.h"
#include "test_sdis_csv_utils.h"

#include <rsys/mem_allocator.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/* ========================================================================== */
/* Physical constants                                                         */
/* ========================================================================== */
#define F2_SRC_POWER       1.0e5
#define F2_SRC_RADIUS      0.3
#define F2_SRC_DISTANCE    10.0

#define F2_SPHERE_RADIUS   0.01
#define F2_SPHERE_TREF     320.0
#define F2_T_RAD           300.0
#define F2_T_REF           300.0

#define F2_LAMBDA          1.0
#define F2_RHO             1.0
#define F2_CP              1.0
#define F2_DELTA           (2.0 * F2_SPHERE_RADIUS / 10.0)

#define F2_NREALS          10000
#define F2_LDIFF           50.0    /* W/m^2/sr */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ========================================================================== */
/* Inline UV sphere mesh generator                                            */
/* ========================================================================== */
/* Generates a UV sphere at origin with given radius, nlon×nlat divisions.
 * Vertex count: nlon*(nlat-1) + 2  (poles)
 * Triangle count: 2*nlon*(nlat-1)
 * Caller must free positions and indices with mem_rm(). */

#define F2_NLON  128
#define F2_NLAT  64
#define F2_NVERTS (F2_NLON * (F2_NLAT - 1) + 2)
#define F2_NTRIS  (2 * F2_NLON * (F2_NLAT - 1))

static double   f2_positions[F2_NVERTS * 3];
static size_t   f2_indices[F2_NTRIS * 3];
static size_t   f2_nverts = 0;
static size_t   f2_ntris = 0;

static void
f2_generate_sphere(double radius)
{
  size_t ilon, ilat, v = 0, t = 0;
  size_t south_pole, north_pole;

  /* South pole */
  south_pole = v;
  f2_positions[v * 3 + 0] = 0.0;
  f2_positions[v * 3 + 1] = 0.0;
  f2_positions[v * 3 + 2] = -radius;
  v++;

  /* Interior latitude rings */
  for(ilat = 1; ilat < (size_t)F2_NLAT; ilat++) {
    double phi = M_PI * (double)ilat / (double)F2_NLAT - M_PI / 2.0;
    double cosphi = cos(phi);
    double sinphi = sin(phi);
    for(ilon = 0; ilon < (size_t)F2_NLON; ilon++) {
      double theta = 2.0 * M_PI * (double)ilon / (double)F2_NLON;
      f2_positions[v * 3 + 0] = radius * cosphi * cos(theta);
      f2_positions[v * 3 + 1] = radius * cosphi * sin(theta);
      f2_positions[v * 3 + 2] = radius * sinphi;
      v++;
    }
  }

  /* North pole */
  north_pole = v;
  f2_positions[v * 3 + 0] = 0.0;
  f2_positions[v * 3 + 1] = 0.0;
  f2_positions[v * 3 + 2] = radius;
  v++;

  f2_nverts = v;
  CHK(f2_nverts == F2_NVERTS);

  /* Triangles: south pole cap */
  for(ilon = 0; ilon < (size_t)F2_NLON; ilon++) {
    size_t next = (ilon + 1) % F2_NLON;
    f2_indices[t * 3 + 0] = south_pole;
    f2_indices[t * 3 + 1] = 1 + ilon;         /* ring 1 */
    f2_indices[t * 3 + 2] = 1 + next;
    t++;
  }

  /* Triangles: middle bands */
  for(ilat = 1; ilat < (size_t)F2_NLAT - 1; ilat++) {
    size_t base0 = 1 + (ilat - 1) * F2_NLON;
    size_t base1 = 1 + ilat * F2_NLON;
    for(ilon = 0; ilon < (size_t)F2_NLON; ilon++) {
      size_t next = (ilon + 1) % F2_NLON;
      /* Lower triangle */
      f2_indices[t * 3 + 0] = base0 + ilon;
      f2_indices[t * 3 + 1] = base1 + ilon;
      f2_indices[t * 3 + 2] = base0 + next;
      t++;
      /* Upper triangle */
      f2_indices[t * 3 + 0] = base0 + next;
      f2_indices[t * 3 + 1] = base1 + ilon;
      f2_indices[t * 3 + 2] = base1 + next;
      t++;
    }
  }

  /* Triangles: north pole cap */
  {
    size_t base_last = 1 + ((size_t)F2_NLAT - 2) * F2_NLON;
    for(ilon = 0; ilon < (size_t)F2_NLON; ilon++) {
      size_t next = (ilon + 1) % F2_NLON;
      f2_indices[t * 3 + 0] = base_last + ilon;
      f2_indices[t * 3 + 1] = north_pole;
      f2_indices[t * 3 + 2] = base_last + next;
      t++;
    }
  }

  f2_ntris = t;
  CHK(f2_ntris == F2_NTRIS);
}

/* ========================================================================== */
/* Scene callbacks                                                            */
/* ========================================================================== */
struct f2_scene_ctx {
  struct sdis_interface* interf;
};

static void
f2_get_indices(const size_t itri, size_t ids[3], void* ctx)
{
  (void)ctx;
  CHK(itri < f2_ntris);
  ids[0] = f2_indices[itri * 3 + 0];
  ids[1] = f2_indices[itri * 3 + 1];
  ids[2] = f2_indices[itri * 3 + 2];
}

static void
f2_get_position(const size_t ivert, double pos[3], void* ctx)
{
  (void)ctx;
  CHK(ivert < f2_nverts);
  pos[0] = f2_positions[ivert * 3 + 0];
  pos[1] = f2_positions[ivert * 3 + 1];
  pos[2] = f2_positions[ivert * 3 + 2];
}

static void
f2_get_interface(const size_t itri, struct sdis_interface** bound, void* ctx)
{
  struct f2_scene_ctx* sc = ctx;
  CHK(itri < f2_ntris && bound);
  *bound = sc->interf;
}

/* ========================================================================== */
/* Media shaders                                                              */
/* ========================================================================== */
static double
f2_get_volumic_mass(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{ (void)vtx; (void)data; return F2_RHO; }

static double
f2_get_calorific_capacity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{ (void)vtx; (void)data; return F2_CP; }

static double
f2_get_temperature(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{ (void)vtx; (void)data; return SDIS_TEMPERATURE_NONE; }

static double
f2_get_thermal_conductivity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{ (void)vtx; (void)data; return F2_LAMBDA; }

static double
f2_get_delta(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{ (void)vtx; (void)data; return F2_DELTA; }

/* ========================================================================== */
/* Interface shader                                                           */
/* ========================================================================== */
static double
f2_interf_get_emissivity
  (const struct sdis_interface_fragment* frag,
   const unsigned source_id,
   struct sdis_data* data)
{
  (void)frag; (void)source_id; (void)data;
  return 1.0;
}

static double
f2_interf_get_ref_temperature
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  (void)frag; (void)data;
  return F2_SPHERE_TREF;
}

/* ========================================================================== */
/* Radiative environment (T_rad=300K)                                         */
/* ========================================================================== */
static double
f2_radenv_get_temperature
  (const struct sdis_radiative_ray* ray, struct sdis_data* data)
{ (void)ray; (void)data; return F2_T_RAD; }

static double
f2_radenv_get_reference
  (const struct sdis_radiative_ray* ray, struct sdis_data* data)
{ (void)ray; (void)data; return F2_T_REF; }

/* ========================================================================== */
/* Source (with mutable diffuse radiance)                                     */
/* ========================================================================== */
static double f2_Ldiff = 0.0; /* mutable, set before each sub-test */

static void
f2_source_get_position
  (const double time, double pos[3], struct sdis_data* data)
{
  (void)time; (void)data;
  pos[0] = F2_SRC_DISTANCE;
  pos[1] = 0.0;
  pos[2] = 0.0;
}

static double
f2_source_get_power
  (const double time, struct sdis_data* data)
{
  (void)time; (void)data;
  return F2_SRC_POWER;
}

static double
f2_source_get_diffuse_radiance
  (const double time, const double dir[3], struct sdis_data* data)
{
  (void)time; (void)dir;
  CHK(data != NULL);
  return *(const double*)sdis_data_cget(data);
}

/* ========================================================================== */
/* Analytic temperature                                                       */
/* ========================================================================== */
static double
f2_analytic_temperature(double Ldiff)
{
  double q = F2_SRC_POWER / (4.0 * M_PI * F2_SRC_DISTANCE * F2_SRC_DISTANCE);
  double T_rad4 = F2_T_RAD * F2_T_RAD * F2_T_RAD * F2_T_RAD;
  return pow((q / 4.0 + Ldiff * M_PI) / BOLTZMANN_CONSTANT + T_rad4, 0.25);
}

/* ========================================================================== */
/* Test body                                                                  */
/* ========================================================================== */
int
main(int argc, char** argv)
{
  struct sdis_device* dev = NULL;
  struct sdis_data* data = NULL;
  struct sdis_medium* fluid = NULL;
  struct sdis_medium* solid = NULL;
  struct sdis_interface* interf = NULL;
  struct sdis_radiative_env* radenv = NULL;
  struct sdis_source* source = NULL;
  struct sdis_scene* scn = NULL;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_solid_shader solid_shader = DUMMY_SOLID_SHADER;
  struct sdis_fluid_shader fluid_shader = DUMMY_FLUID_SHADER;
  struct sdis_radiative_env_shader radenv_shader = SDIS_RADIATIVE_ENV_SHADER_NULL;
  struct sdis_spherical_source_shader source_shader
    = SDIS_SPHERICAL_SOURCE_SHADER_NULL;
  struct sdis_interface_shader interf_shader = SDIS_INTERFACE_SHADER_NULL;
  struct f2_scene_ctx scene_ctx;
  double* pLdiff = NULL;
  double T_ref;
  int pass;
  FILE* csv = NULL;
  (void)argc; (void)argv;

  printf("=== WF-F2: External flux + diffuse radiance (wavefront probe) ===\n");

  /* Generate sphere mesh */
  f2_generate_sphere(F2_SPHERE_RADIUS);
  printf("  Sphere mesh: %lu verts, %lu tris\n",
    (unsigned long)f2_nverts, (unsigned long)f2_ntris);

  csv = csv_open("F2");

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Fluid (T=NONE) ---- */
  fluid_shader.calorific_capacity = f2_get_calorific_capacity;
  fluid_shader.volumic_mass = f2_get_volumic_mass;
  fluid_shader.temperature = f2_get_temperature;
  OK(sdis_fluid_create(dev, &fluid_shader, NULL, &fluid));

  /* ---- Solid (lambda=1, small sphere) ---- */
  solid_shader.calorific_capacity = f2_get_calorific_capacity;
  solid_shader.thermal_conductivity = f2_get_thermal_conductivity;
  solid_shader.volumic_mass = f2_get_volumic_mass;
  solid_shader.delta = f2_get_delta;
  solid_shader.temperature = f2_get_temperature;
  OK(sdis_solid_create(dev, &solid_shader, NULL, &solid));

  /* ---- Interface (fluid/solid, eps=1, Tref=320K) ---- */
  interf_shader.front.emissivity = f2_interf_get_emissivity;
  interf_shader.front.reference_temperature = f2_interf_get_ref_temperature;
  OK(sdis_interface_create(dev, fluid, solid, &interf_shader, NULL, &interf));

  OK(sdis_medium_ref_put(fluid));
  OK(sdis_medium_ref_put(solid));

  /* ---- Radiative environment ---- */
  radenv_shader.temperature = f2_radenv_get_temperature;
  radenv_shader.reference_temperature = f2_radenv_get_reference;
  OK(sdis_radiative_env_create(dev, &radenv_shader, NULL, &radenv));

  /* ---- Source (with mutable diffuse radiance via sdis_data) ---- */
  OK(sdis_data_create(dev, sizeof(double), ALIGNOF(double), NULL, &data));
  pLdiff = sdis_data_get(data);
  *pLdiff = 0.0;

  source_shader.position = f2_source_get_position;
  source_shader.power = f2_source_get_power;
  source_shader.diffuse_radiance = f2_source_get_diffuse_radiance;
  source_shader.radius = F2_SRC_RADIUS;
  OK(sdis_spherical_source_create(dev, &source_shader, data, &source));
  OK(sdis_data_ref_put(data));

  /* Get mutable pointer via source */
  pLdiff = sdis_data_get(sdis_source_get_data(source));

  /* ---- Scene ---- */
  scene_ctx.interf = interf;

  scn_args.get_indices = f2_get_indices;
  scn_args.get_interface = f2_get_interface;
  scn_args.get_position = f2_get_position;
  scn_args.nprimitives = f2_ntris;
  scn_args.nvertices = f2_nverts;
  scn_args.t_range[0] = F2_T_REF < F2_SPHERE_TREF ? F2_T_REF : F2_SPHERE_TREF;
  scn_args.t_range[1] = F2_T_REF > F2_SPHERE_TREF ? F2_T_REF : F2_SPHERE_TREF;
  scn_args.source = source;
  scn_args.radenv = radenv;
  scn_args.context = &scene_ctx;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  OK(sdis_interface_ref_put(interf));
  OK(sdis_radiative_env_ref_put(radenv));
  OK(sdis_source_ref_put(source));

  /* ---- Test with Ldiff = 50 W/m^2/sr ---- */
  {
    struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
    struct sdis_estimator *est_wf = NULL, *est_df = NULL;

    *pLdiff = F2_LDIFF;
    T_ref = f2_analytic_temperature(F2_LDIFF);

    printf("  Ldiff = %.1f W/m^2/sr, analytic Ts = %.4f K\n",
      F2_LDIFF, T_ref);

    args.nrealisations = F2_NREALS;
    args.position[0] = 0.0;
    args.position[1] = 0.0;
    args.position[2] = 0.0;
    args.picard_order = 1;
    args.diff_algo = SDIS_DIFFUSION_DELTA_SPHERE;

    OK(sdis_solve_wavefront_probe(scn, &args, &est_wf));

    pass = p0_compare_analytic(est_wf, T_ref, P0_TOL_SIGMA);

    /* CSV: primary DS row + complementary WoS variant */
    {
      struct sdis_mc mc_csv;
      OK(sdis_estimator_get_temperature(est_wf, &mc_csv));
      csv_row(csv, "F2", "Ldiff=50", "gpu_wf", "DS",
              0.0, 0.0, 0.0, INF, 1, F2_NREALS,
              mc_csv.E, mc_csv.SE, T_ref);

    }

    if(P0_ENABLE_DIAG) {
      OK(sdis_solve_probe(scn, &args, &est_df));
    }

    p0_print_probe_result(0.0, est_wf, est_df, T_ref);

    OK(sdis_estimator_ref_put(est_wf));
    if(est_df) OK(sdis_estimator_ref_put(est_df));
  }

  csv_close(csv);
  CHK(pass);
  printf("WF-F2: PASS\n");

  /* ---- Cleanup ---- */
  OK(sdis_scene_ref_put(scn));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
