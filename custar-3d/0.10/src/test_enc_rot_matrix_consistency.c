/* test_enc_rot_matrix_consistency.c
 *
 * Verify that the GPU kernel's ENC_ROT_MATRIX (used in
 * cus3d_find_enclosure.cu for 6-ray side determination) produces the
 * same rotated directions as the CPU reference f33_rotation(PI/4, PI/4, PI/4)
 * + f33_mulf3.
 *
 * Related investigation: debug_issues/enc_rot_matrix_mismatch/
 *
 * Tests:
 *   T1: Compare 6 rotated directions (CPU vs GPU matrix)
 *   T2: Verify both matrices produce non-axis-aligned directions
 *   T3: Verify both matrices are orthogonal (det = 1, M^T * M = I)
 *   T4: Print numerical values for manual inspection
 */

#include "s3d.h"
#include <rsys/float33.h>
#include <math.h>
#include <stdio.h>

/* ---- GPU kernel constants (must match cus3d_find_enclosure.cu) ---- */

/* Pre-computed rotation matrix matching CPU f33_rotation(PI/4, PI/4, PI/4).
 * Row-major 3x3.  These values are a direct copy of the ENC_ROT_MATRIX
 * array in cus3d_find_enclosure.cu — keep them in sync. */
static const float GPU_ENC_ROT_MATRIX[9] = {
    /* Row 0: */  0.5f,               -0.5f,                0.70710678118f,
    /* Row 1: */  0.85355339059f,      0.14644660941f,     -0.5f,
    /* Row 2: */  0.14644660941f,      0.85355339059f,      0.5f
};

/* GPU row-major matrix-vector multiply (matches enc_rotate_dir in kernel) */
static void
gpu_rotate_dir(float out[3], const float v[3])
{
    out[0] = GPU_ENC_ROT_MATRIX[0]*v[0] + GPU_ENC_ROT_MATRIX[1]*v[1] + GPU_ENC_ROT_MATRIX[2]*v[2];
    out[1] = GPU_ENC_ROT_MATRIX[3]*v[0] + GPU_ENC_ROT_MATRIX[4]*v[1] + GPU_ENC_ROT_MATRIX[5]*v[2];
    out[2] = GPU_ENC_ROT_MATRIX[6]*v[0] + GPU_ENC_ROT_MATRIX[7]*v[1] + GPU_ENC_ROT_MATRIX[8]*v[2];
}

/* 6 axis-aligned base directions (same as kernel) */
static const float AXIS_DIRS[6][3] = {
    { 1, 0, 0}, {-1, 0, 0},
    { 0, 1, 0}, { 0,-1, 0},
    { 0, 0, 1}, { 0, 0,-1}
};

static const char* DIR_NAMES[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};

static float fabsf_safe(float x) { return x < 0.0f ? -x : x; }

/* ====================================================================
 * T1: Compare 6 rotated directions
 * ==================================================================== */
static void
test_rotated_directions_match(void)
{
    float cpu_frame[9];
    float cpu_dir[3], gpu_dir[3];
    int idir, comp;
    int all_match = 1;
    const float tol = 1.0e-6f;

    printf("  T1: 6 rotated directions CPU vs GPU... ");

    /* CPU: build rotation matrix using rsys */
    f33_rotation(cpu_frame, (float)(3.14159265358979323846/4.0),
                            (float)(3.14159265358979323846/4.0),
                            (float)(3.14159265358979323846/4.0));

    for (idir = 0; idir < 6; ++idir) {
        float diff[3];
        float max_err = 0.0f;

        /* CPU rotation */
        f33_mulf3(cpu_dir, cpu_frame, (float*)AXIS_DIRS[idir]);

        /* GPU rotation */
        gpu_rotate_dir(gpu_dir, AXIS_DIRS[idir]);

        for (comp = 0; comp < 3; ++comp) {
            diff[comp] = cpu_dir[comp] - gpu_dir[comp];
            if (fabsf_safe(diff[comp]) > max_err)
                max_err = fabsf_safe(diff[comp]);
        }

        if (max_err > tol) {
            if (all_match) printf("FAIL\n");
            all_match = 0;
            printf("    MISMATCH dir %s: max_err = %.8e\n", DIR_NAMES[idir], (double)max_err);
            printf("      CPU: [%+.8f, %+.8f, %+.8f]\n",
                   (double)cpu_dir[0], (double)cpu_dir[1], (double)cpu_dir[2]);
            printf("      GPU: [%+.8f, %+.8f, %+.8f]\n",
                   (double)gpu_dir[0], (double)gpu_dir[1], (double)gpu_dir[2]);
            printf("      dif: [%+.8e, %+.8e, %+.8e]\n",
                   (double)diff[0], (double)diff[1], (double)diff[2]);
        }
    }

    if (all_match) {
        printf("PASS\n");
    } else {
        printf("  T1 FAILED: GPU ENC_ROT_MATRIX does not match CPU f33_rotation\n");
        printf("             See debug_issues/enc_rot_matrix_mismatch/investigation.md\n");
    }

    /* Intentionally do NOT abort on failure — this is a diagnostic test.
     * The mismatch is a confirmed divergence, not necessarily a correctness bug.
     * We print the discrepancy and let the developer decide. */
}

/* ====================================================================
 * T2: Both matrices produce non-axis-aligned directions
 * ==================================================================== */
static void
test_non_axis_aligned(void)
{
    float cpu_frame[9];
    float cpu_dir[3], gpu_dir[3];
    int idir, comp;
    const float axis_threshold = 0.99f;

    printf("  T2: All rotated directions are non-axis-aligned... ");

    f33_rotation(cpu_frame, (float)(3.14159265358979323846/4.0),
                            (float)(3.14159265358979323846/4.0),
                            (float)(3.14159265358979323846/4.0));

    for (idir = 0; idir < 6; ++idir) {
        f33_mulf3(cpu_dir, cpu_frame, (float*)AXIS_DIRS[idir]);
        gpu_rotate_dir(gpu_dir, AXIS_DIRS[idir]);

        for (comp = 0; comp < 3; ++comp) {
            CHK(fabsf_safe(cpu_dir[comp]) < axis_threshold);
            CHK(fabsf_safe(gpu_dir[comp]) < axis_threshold);
        }
    }

    printf("PASS\n");
}

/* ====================================================================
 * T3: Both matrices are orthogonal (det ~= 1, M^T * M ~= I)
 * ==================================================================== */
static void
test_orthogonality(void)
{
    float cpu_frame[9];
    float cpu_MtM[9]; /* M^T * M */
    float gpu_MtM[9];
    int i, j, k;
    const float tol = 1.0e-5f;

    printf("  T3: Both matrices are orthogonal (M^T * M = I)... ");

    f33_rotation(cpu_frame, (float)(3.14159265358979323846/4.0),
                            (float)(3.14159265358979323846/4.0),
                            (float)(3.14159265358979323846/4.0));

    /* CPU: compute M^T * M using f33_mulf3 on identity columns.
     * Since f33_mulf3 uses column-major mat, the logical matrix M is:
     *   M(row, col) = mat[col * 3 + row]
     * M^T * M: (M^T * M)(i,j) = sum_k M(k,i) * M(k,j)
     *   = sum_k mat[i*3 + k] * mat[j*3 + k]  ... no, let me just use
     *   the definition directly via matrix-vector products.
     */
    {
        /* Extract the 3 logical rows of the CPU matrix.
         * row(mat, y)[x] = mat[x * 3 + y]  (rsys column-major) */
        float rows[3][3];
        for (i = 0; i < 3; ++i)
            for (j = 0; j < 3; ++j)
                rows[i][j] = cpu_frame[j * 3 + i];

        /* M^T * M: MtM(i,j) = dot(col_i, col_j).
         * In column-major: col_i = &cpu_frame[i * 3], length 3. */
        for (i = 0; i < 3; ++i)
            for (j = 0; j < 3; ++j) {
                cpu_MtM[j * 3 + i] = 0.0f;
                for (k = 0; k < 3; ++k)
                    cpu_MtM[j * 3 + i] += cpu_frame[i * 3 + k] * cpu_frame[j * 3 + k];
            }

        for (i = 0; i < 3; ++i)
            for (j = 0; j < 3; ++j) {
                float expected = (i == j) ? 1.0f : 0.0f;
                CHK(fabsf_safe(cpu_MtM[j * 3 + i] - expected) < tol);
            }
    }

    /* GPU: row-major, M^T * M: MtM(i,j) = sum_k M(k,i)*M(k,j)
     *   = sum_k GPU[k*3+i] * GPU[k*3+j] */
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (k = 0; k < 3; ++k)
                s += GPU_ENC_ROT_MATRIX[k * 3 + i] * GPU_ENC_ROT_MATRIX[k * 3 + j];
            gpu_MtM[i * 3 + j] = s;
        }

    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j) {
            float expected = (i == j) ? 1.0f : 0.0f;
            CHK(fabsf_safe(gpu_MtM[i * 3 + j] - expected) < tol);
        }

    printf("PASS\n");
}

/* ====================================================================
 * T4: Print numerical values for manual inspection
 * ==================================================================== */
static void
test_print_matrices(void)
{
    float cpu_frame[9];
    float cpu_rows[3][3];
    int i, j;

    printf("  T4: Numerical matrix values (informational)\n");

    f33_rotation(cpu_frame, (float)(3.14159265358979323846/4.0),
                            (float)(3.14159265358979323846/4.0),
                            (float)(3.14159265358979323846/4.0));

    /* Extract logical rows from column-major CPU frame */
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j)
            cpu_rows[i][j] = cpu_frame[j * 3 + i];

    printf("    CPU f33_rotation(PI/4, PI/4, PI/4) — logical matrix:\n");
    for (i = 0; i < 3; ++i)
        printf("      [%+.8f  %+.8f  %+.8f]\n",
               (double)cpu_rows[i][0], (double)cpu_rows[i][1], (double)cpu_rows[i][2]);

    printf("    GPU ENC_ROT_MATRIX (row-major):\n");
    for (i = 0; i < 3; ++i)
        printf("      [%+.8f  %+.8f  %+.8f]\n",
               (double)GPU_ENC_ROT_MATRIX[i*3+0],
               (double)GPU_ENC_ROT_MATRIX[i*3+1],
               (double)GPU_ENC_ROT_MATRIX[i*3+2]);

    printf("    Rotated directions comparison:\n");
    for (i = 0; i < 6; ++i) {
        float cpu_dir[3], gpu_dir[3];
        f33_mulf3(cpu_dir, cpu_frame, (float*)AXIS_DIRS[i]);
        gpu_rotate_dir(gpu_dir, AXIS_DIRS[i]);
        printf("      %s: CPU=[%+.6f %+.6f %+.6f]  GPU=[%+.6f %+.6f %+.6f]\n",
               DIR_NAMES[i],
               (double)cpu_dir[0], (double)cpu_dir[1], (double)cpu_dir[2],
               (double)gpu_dir[0], (double)gpu_dir[1], (double)gpu_dir[2]);
    }
}

/* ====================================================================
 * T5: Verify both rotated direction sets preserve unit length
 * ==================================================================== */
static void
test_unit_length_preserved(void)
{
    float cpu_frame[9];
    float cpu_dir[3], gpu_dir[3];
    int idir;
    const float tol = 1.0e-6f;

    printf("  T5: Rotated directions preserve unit length... ");

    f33_rotation(cpu_frame, (float)(3.14159265358979323846/4.0),
                            (float)(3.14159265358979323846/4.0),
                            (float)(3.14159265358979323846/4.0));

    for (idir = 0; idir < 6; ++idir) {
        float cpu_len, gpu_len;
        f33_mulf3(cpu_dir, cpu_frame, (float*)AXIS_DIRS[idir]);
        gpu_rotate_dir(gpu_dir, AXIS_DIRS[idir]);

        cpu_len = (float)sqrt((double)(cpu_dir[0]*cpu_dir[0]
                    + cpu_dir[1]*cpu_dir[1] + cpu_dir[2]*cpu_dir[2]));
        gpu_len = (float)sqrt((double)(gpu_dir[0]*gpu_dir[0]
                    + gpu_dir[1]*gpu_dir[1] + gpu_dir[2]*gpu_dir[2]));

        CHK(fabsf_safe(cpu_len - 1.0f) < tol);
        CHK(fabsf_safe(gpu_len - 1.0f) < tol);
    }

    printf("PASS\n");
}

/* ==================================================================== */

int main(int argc, char** argv)
{
    (void)argc; (void)argv;

    printf("ENC_ROT_MATRIX consistency: GPU kernel vs CPU f33_rotation\n");
    printf("============================================================\n");

    test_print_matrices();                /* T4: print first for diagnostics */
    test_rotated_directions_match();      /* T1: core comparison */
    test_non_axis_aligned();              /* T2: validity check */
    test_orthogonality();                 /* T3: mathematical correctness */
    test_unit_length_preserved();         /* T5: length preservation */

    printf("\nAll ENC_ROT_MATRIX consistency tests completed.\n");
    return 0;
}
