/* Copyright (C) 2016-2025 |Méso|Star> (contact@meso-star.com)
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

/* Wavefront step function library — declarations.
 *
 * All functions that implement individual path-state transitions or ray
 * setup helpers are declared here with LOCAL_SYM visibility.  They live
 * in sdis_wf_steps.c and are linked into the sdis library, accessible to
 * any translation unit within the library (wavefront.c, persistent_wavefront.c,
 * tests) but hidden from the public ABI.
 *
 * This header is the replacement for the `#include "sdis_solve_wavefront.c"`
 * hack that previously gave persistent_wavefront.c and test files access
 * to the static step functions.
 */

#ifndef SDIS_WF_STEPS_H
#define SDIS_WF_STEPS_H

#include "sdis_wf_state.h"  /* struct path_state, struct path_ray_request    */
#include "sdis.h"           /* LOCAL_SYM, res_T                              */
#include <star/s3d.h>       /* struct s3d_hit                                */

/* Forward declarations — full definitions live in internal headers */
struct sdis_scene;
struct sdis_interface;
struct sdis_interface_fragment;

/*******************************************************************************
 * Utility functions
 ******************************************************************************/

/* Check that an interface fragment is on the fluid side (copied from
 * sdis_heat_path_radiative_Xd.h because it is static in that header). */
extern LOCAL_SYM res_T
wf_check_interface
  (const struct sdis_interface* interf,
   const struct sdis_interface_fragment* frag,
   const int verbose);

/* Sample reinjection direction around hit normal (replicates the static
 * sample_reinjection_dir_3d from sdis_heat_path_boundary_Xd_c.h). */
extern LOCAL_SYM void
wf_sample_reinjection_dir_3d
  (const struct rwalk* rwalk,
   struct ssp_rng* rng,
   float dir[3]);

/*******************************************************************************
 * Ray setup helpers — populate path_state::ray_req for the next batch
 ******************************************************************************/

extern LOCAL_SYM void
setup_radiative_trace_ray(struct path_state* p, struct sdis_scene* scn);

extern LOCAL_SYM void
setup_delta_sphere_rays(struct path_state* p, struct sdis_scene* scn);

extern LOCAL_SYM void
setup_convective_startup_ray(struct path_state* p);

extern LOCAL_SYM void
setup_ss_reinject_rays(struct path_state* p);

/*******************************************************************************
 * Step functions — each advances a path by exactly one logical step
 ******************************************************************************/

/* PATH_INIT: emit first radiative trace ray */
extern LOCAL_SYM res_T
step_init(struct path_state* p, struct sdis_scene* scn);

/* PATH_RAD_TRACE_PENDING: process hit / BRDF decision */
extern LOCAL_SYM res_T
step_radiative_trace
  (struct path_state* p,
   struct sdis_scene* scn,
   const struct s3d_hit* trace_hit);

/* PATH_COUPLED_BOUNDARY: boundary_path logic (no ray needed) */
extern LOCAL_SYM res_T
step_boundary(struct path_state* p, struct sdis_scene* scn);

/* PATH_COUPLED_CONDUCTIVE: delta_sphere entry / loop */
extern LOCAL_SYM res_T
step_conductive(struct path_state* p, struct sdis_scene* scn);

/* PATH_COUPLED_COND_DS_PENDING: process 2-ray delta sphere results */
extern LOCAL_SYM res_T
step_conductive_ds_process
  (struct path_state* p,
   struct sdis_scene* scn,
   const struct s3d_hit* hit0,
   const struct s3d_hit* hit1);

/* PATH_COUPLED_CONVECTIVE: convective_path (may need startup ray) */
extern LOCAL_SYM res_T
step_convective(struct path_state* p, struct sdis_scene* scn);

/* PATH_COUPLED_RADIATIVE: bounce into radiative from boundary */
extern LOCAL_SYM res_T
step_coupled_radiative_begin(struct path_state* p, struct sdis_scene* scn);

/*******************************************************************************
 * B-4 M10: Point-in-enclosure via BVH closest primitive
 ******************************************************************************/

/* Submit a point-in-enclosure query via the BVH closest primitive kernel.
 * Sets enc_locate.query_pos and transitions to PATH_ENC_LOCATE_PENDING.
 * The wavefront pool will collect these requests into a batch. */
extern LOCAL_SYM void
step_enc_locate_submit(struct path_state* p,
                       const double pos[3],
                       enum path_phase return_state);

/* Process the result of a batch enc_locate query.
 * Resolves prim_id + side → enc_id via scene prim_props and transitions
 * to the saved return_state. */
extern LOCAL_SYM res_T
step_enc_locate_result(struct path_state* p, struct sdis_scene* scn);

/*******************************************************************************
 * B-4 M1-v2: 6-ray enclosure query (replaces M10 at call sites)
 *
 * Emits 6 axis-aligned rays (PI/4 rotated) to determine the enclosure
 * containing a query point.  First valid hit resolves the enclosure ID.
 * If all 6 miss, a single fallback random-direction ray is emitted as
 * a batched operation (avoids synchronous brute-force).
 ******************************************************************************/

/* Submit: build 6 PI/4-rotated axis-aligned directions + set up ray request.
 * Transitions to PATH_ENC_QUERY_EMIT (ray-pending). */
extern LOCAL_SYM void
step_enc_query_emit(struct path_state* p,
                    const double pos[3],
                    enum path_phase return_state);

/* Resolve: process 6 hit results, pick first valid.
 * Called from advance_one_step_with_ray when phase==PATH_ENC_QUERY_EMIT.
 * If resolved → return_state; if all invalid → PATH_ENC_QUERY_FB_EMIT. */
extern LOCAL_SYM res_T
step_enc_query_resolve(struct path_state* p, struct sdis_scene* scn);

/* Fallback resolve: process 1 fallback hit result.
 * Called from advance_one_step_with_ray when phase==PATH_ENC_QUERY_FB_EMIT.
 * If resolved → return_state; if still invalid → synchronous fallback. */
extern LOCAL_SYM res_T
step_enc_query_fb_resolve(struct path_state* p, struct sdis_scene* scn);

/*******************************************************************************
 * B-4 M3: Solid/solid reinjection batch state machine
 ******************************************************************************/

/* Prepare and emit 4 reinjection rays */
extern LOCAL_SYM res_T
step_bnd_ss_reinject_sample(struct path_state* p, struct sdis_scene* scn);

/* Resolve 2-direction reinjection from hit results (pure compute helper) */
extern LOCAL_SYM void
resolve_reinjection_from_hits
  (const struct s3d_hit* hit0,
   const struct s3d_hit* hit1,
   const float dir0[3],
   const float dir1[3],
   unsigned enc0_id,
   unsigned enc1_id,
   unsigned solid_enc_id,
   double distance,
   float out_dir[3],
   float* out_dst,
   struct s3d_hit* out_hit);

/* Process 4-ray results for solid/solid reinjection */
extern LOCAL_SYM res_T
step_bnd_ss_reinject_process
  (struct path_state* p,
   struct sdis_scene* scn,
   const struct s3d_hit* hit_frt0,
   const struct s3d_hit* hit_frt1,
   const struct s3d_hit* hit_bck0,
   const struct s3d_hit* hit_bck1);

/* ENC query result for reinjection verification */
extern LOCAL_SYM res_T
step_bnd_ss_reinject_enc_result(struct path_state* p, struct sdis_scene* scn);

/* Probability choice + solid_reinjection */
extern LOCAL_SYM res_T
step_bnd_ss_reinject_decide(struct path_state* p, struct sdis_scene* scn);

/*******************************************************************************
 * B-4 M5: Solid/fluid picard1 batch state machine
 ******************************************************************************/

/* Ray setup helper for SF reinjection (2 rays: dir0 + reflect(dir0)) */
extern LOCAL_SYM void
setup_sf_reinject_rays(struct path_state* p);

/* PATH_BND_SF_REINJECT_SAMPLE: prepare interface data + emit 2 rays */
extern LOCAL_SYM res_T
step_bnd_sf_reinject_sample(struct path_state* p, struct sdis_scene* scn);

/* Process 2-ray results for solid/fluid reinjection */
extern LOCAL_SYM res_T
step_bnd_sf_reinject_process
  (struct path_state* p,
   struct sdis_scene* scn,
   const struct s3d_hit* hit0,
   const struct s3d_hit* hit1);

/* PATH_BND_SF_REINJECT_ENC: ENC query result for reinjection verification */
extern LOCAL_SYM res_T
step_bnd_sf_reinject_enc_result(struct path_state* p, struct sdis_scene* scn);

/* PATH_BND_SF_PROB_DISPATCH: compute probabilities + null-collision dispatch */
extern LOCAL_SYM res_T
step_bnd_sf_prob_dispatch(struct path_state* p, struct sdis_scene* scn);

/* PATH_BND_SF_NULLCOLL_RAD_TRACE: process radiative trace hit in sub-path */
extern LOCAL_SYM res_T
step_bnd_sf_nullcoll_rad_trace
  (struct path_state* p,
   struct sdis_scene* scn,
   const struct s3d_hit* hit);

/* PATH_BND_SF_NULLCOLL_DECIDE: accept/reject radiative path (pure compute) */
extern LOCAL_SYM res_T
step_bnd_sf_nullcoll_decide(struct path_state* p, struct sdis_scene* scn);

/*******************************************************************************
 * B-4 M8: PicardN recursive stack state machine
 ******************************************************************************/

/* PATH_BND_SFN_PROB_DISPATCH: picardN null-collision probability dispatch.
 * Reuses bnd_sf reinjection data.  h_hat==0 → first entry (compute coeffs),
 * h_hat>0 → null-collision dispatch (conv/cond/rad). */
extern LOCAL_SYM res_T
step_bnd_sfn_prob_dispatch(struct path_state* p, struct sdis_scene* scn);

/* PATH_BND_SFN_RAD_TRACE: process radiative sub-path trace hit.
 * Functionally identical to step_bnd_sf_nullcoll_rad_trace but transitions
 * to PATH_BND_SFN_RAD_DONE on completion instead of SF_NULLCOLL_DECIDE. */
extern LOCAL_SYM res_T
step_bnd_sfn_rad_trace(struct path_state* p, struct sdis_scene* scn,
                       const struct s3d_hit* hit);

/* PATH_BND_SFN_RAD_DONE: radiative sub-path completed.
 * Saves rwalk_s/T_s, computes initial h_radi_min check, then starts the
 * COMPUTE_TEMPERATURE chain or early-accepts. */
extern LOCAL_SYM res_T
step_bnd_sfn_rad_done(struct path_state* p, struct sdis_scene* scn);

/* PATH_BND_SFN_COMPUTE_Ti: compute i-th temperature sample.
 * If T.done → use value directly, else push stack and start sub-path. */
extern LOCAL_SYM res_T
step_bnd_sfn_compute_Ti(struct path_state* p, struct sdis_scene* scn);

/* PATH_BND_SFN_COMPUTE_Ti_RESUME: sub-path returned, pop stack frame. */
extern LOCAL_SYM res_T
step_bnd_sfn_compute_Ti_resume(struct path_state* p, struct sdis_scene* scn);

/* PATH_BND_SFN_CHECK_PMIN_PMAX: early accept/reject based on h_radi bounds. */
extern LOCAL_SYM res_T
step_bnd_sfn_check_pmin_pmax(struct path_state* p, struct sdis_scene* scn);

/*******************************************************************************
 * B-4 M4: Delta-sphere conductive fine-grained state machine
 ******************************************************************************/

/* PATH_CND_DS_CHECK_TEMP: finalize init (once) + check known temp + emit DS
 * rays -> PATH_CND_DS_STEP_TRACE */
extern LOCAL_SYM res_T
step_cnd_ds_check_temp(struct path_state* p, struct sdis_scene* scn);

/* PATH_CND_DS_STEP_ENC_VERIFY: set up enc_locate for enclosure verify at
 * pos_next.  Calls step_enc_locate_submit -> PATH_ENC_LOCATE_PENDING. */
extern LOCAL_SYM void
step_cnd_ds_step_enc_verify(struct path_state* p);

/* PATH_CND_DS_STEP_ADVANCE: check enc result + volumic power + time rewind +
 * position update + loop condition check */
extern LOCAL_SYM res_T
step_cnd_ds_step_advance(struct path_state* p, struct sdis_scene* scn);

/*******************************************************************************
 * B-4 M9: Walk on Spheres (WoS) conductive path state machine
 *
 * WoS uses closest_point queries (batched via s3d_batch_cp_context) instead
 * of ray traces for the main geometry query.  Fallback to trace_ray only
 * when check_diffusion_position detects the sampled point crossed a boundary.
 ******************************************************************************/

/* PATH_CND_WOS_CHECK_TEMP: loop top — check temperature known + emit CP query.
 * On first entry: initialise WoS state (enc_id, medium, alpha, props).
 * On re-entry: fetch props at new position, check temperature. */
extern LOCAL_SYM res_T
step_cnd_wos_check_temp(struct path_state* p, struct sdis_scene* scn);

/* PATH_CND_WOS_CLOSEST: submit closest_point query to batch.
 * Sets up s3d_cp_request fields; pool collect will gather these. */
extern LOCAL_SYM void
step_cnd_wos_closest(struct path_state* p);

/* PATH_CND_WOS_CLOSEST_RESULT: process closest_point result.
 * Epsilon-shell → snap to boundary → TIME_TRAVEL.
 * Normal → sample sphere, save candidate pos → DIFFUSION_CHECK. */
extern LOCAL_SYM res_T
step_cnd_wos_closest_result(struct path_state* p, struct sdis_scene* scn);

/* PATH_CND_WOS_DIFFUSION_CHECK: submit batched CP query to validate
 * diffusion candidate position (radius = delta).  Replaces former
 * synchronous wf_check_diffusion_position(). */
extern LOCAL_SYM void
step_cnd_wos_diffusion_check(struct path_state* p);

/* PATH_CND_WOS_DIFFUSION_CHECK_RESULT: process validation CP result.
 * position valid → move, TIME_TRAVEL.
 * position invalid → FALLBACK_TRACE. */
extern LOCAL_SYM res_T
step_cnd_wos_diffusion_check_result(struct path_state* p,
                                     struct sdis_scene* scn);

/* PATH_CND_WOS_FALLBACK_TRACE: emit fallback trace_ray (dir from closest_result).
 * ray_bucket = RAY_BUCKET_RADIATIVE, 1 ray. */
extern LOCAL_SYM void
step_cnd_wos_fallback_trace(struct path_state* p);

/* PATH_CND_WOS_FALLBACK_RESULT: process fallback ray result.
 * miss → snap to cached closest_point hit.
 * hit → setup_hit_rt or snap on enclosure mismatch.
 * Always transitions to TIME_TRAVEL. */
extern LOCAL_SYM res_T
step_cnd_wos_fallback_result(struct path_state* p, struct sdis_scene* scn,
                             const struct s3d_hit* hit);

/* PATH_CND_WOS_TIME_TRAVEL: time rewind + volumic power + loop/exit decision.
 * T->done → PATH_DONE.
 * hit → PATH_BND_DISPATCH (via PATH_COUPLED_BOUNDARY).
 * else → PATH_CND_WOS_CHECK_TEMP (continue loop). */
extern LOCAL_SYM res_T
step_cnd_wos_time_travel(struct path_state* p, struct sdis_scene* scn);

/*******************************************************************************
 * B-4 M7: External net flux batch state machine
 ******************************************************************************/

/* PATH_BND_EXT_CHECK: check if external flux is needed, set up shared state */
extern LOCAL_SYM res_T
step_bnd_ext_check(struct path_state* p, struct sdis_scene* scn);

/* PATH_BND_EXT_DIRECT_TRACE: process shadow ray result (direct contribution) */
extern LOCAL_SYM res_T
step_bnd_ext_direct_result(struct path_state* p, struct sdis_scene* scn,
                           const struct s3d_hit* hit);

/* PATH_BND_EXT_DIFFUSE_RESULT: process diffuse bounce ray result */
extern LOCAL_SYM res_T
step_bnd_ext_diffuse_result(struct path_state* p, struct sdis_scene* scn,
                            const struct s3d_hit* hit);

/* PATH_BND_EXT_DIFFUSE_SHADOW_RESULT: process bounce shadow ray result */
extern LOCAL_SYM res_T
step_bnd_ext_diffuse_shadow_result(struct path_state* p,
                                   struct sdis_scene* scn,
                                   const struct s3d_hit* hit);

/* PATH_BND_EXT_FINALIZE: sum flux contributions, apply to T, return to caller */
extern LOCAL_SYM res_T
step_bnd_ext_finalize(struct path_state* p, struct sdis_scene* scn);

/*******************************************************************************
 * B-4 M6: Convective path + boundary dispatch + Robin post-check
 ******************************************************************************/

/* PATH_CNV_INIT: check known fluid temperature, decide startup ray or loop */
extern LOCAL_SYM res_T
step_cnv_init(struct path_state* p, struct sdis_scene* scn);

/* PATH_CNV_STARTUP_RESULT: process startup ray result, set hit_side */
extern LOCAL_SYM res_T
step_cnv_startup_result(struct path_state* p, struct sdis_scene* scn,
                        const struct s3d_hit* hit);

/* PATH_CNV_SAMPLE_LOOP: null-collision sampling loop (pure compute, no ray) */
extern LOCAL_SYM res_T
step_cnv_sample_loop(struct path_state* p, struct sdis_scene* scn);

/* PATH_BND_DISPATCH: Dirichlet check + 3-way dispatch */
extern LOCAL_SYM res_T
step_bnd_dispatch(struct path_state* p, struct sdis_scene* scn);

/* PATH_BND_POST_ROBIN_CHECK: Robin boundary condition post-check */
extern LOCAL_SYM res_T
step_bnd_post_robin_check(struct path_state* p, struct sdis_scene* scn);

/*******************************************************************************
 * Dispatch — advance one path by one step
 ******************************************************************************/

/* Advance without ray.  Returns *advanced=1 if the path was advanced. */
extern LOCAL_SYM res_T
advance_one_step_no_ray
  (struct path_state* p, struct sdis_scene* scn, int* advanced);

/* Advance after receiving ray trace result(s) */
extern LOCAL_SYM res_T
advance_one_step_with_ray
  (struct path_state* p,
   struct sdis_scene* scn,
   const struct s3d_hit* hit0,
   const struct s3d_hit* hit1);

#endif /* SDIS_WF_STEPS_H */
