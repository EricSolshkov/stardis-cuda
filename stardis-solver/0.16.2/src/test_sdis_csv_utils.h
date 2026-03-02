/* test_sdis_csv_utils.h — CSV data output for architecture consistency proof.
 *
 * Usage:
 *   FILE* csv = csv_open("A3");          // reads SDIS_CSV_DIR env var
 *   csv_row(csv, "A3", "R=0.01", "gpu_wf", "DS",
 *           x, y, z, INFINITY, 1, 10000, mc.E, mc.SE, ref);
 *   csv_close(csv);
 *
 * If SDIS_CSV_DIR is not set, csv_open returns NULL and csv_row is a no-op.
 * Pure C89, header-only, static functions. */

#ifndef TEST_SDIS_CSV_UTILS_H
#define TEST_SDIS_CSV_UTILS_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define CSV_HEADER \
  "test_id,sub_id,solver,diff_algo,probe_x,probe_y,probe_z," \
  "time,picard_order,nrealisations,E,SE,ref,sigma,pass\n"

static FILE*
csv_open(const char* test_id)
{
  const char* dir;
  char path[512];
  FILE* fp;

#if defined(_MSC_VER)
  dir = NULL;
  {
    char buf[512];
    size_t len = 0;
    if(getenv_s(&len, buf, sizeof(buf), "SDIS_CSV_DIR") != 0 || len == 0)
      return NULL;
    dir = buf;
    _snprintf(path, sizeof(path), "%s\\%s.csv", dir, test_id);
  }
#else
  dir = getenv("SDIS_CSV_DIR");
  if(!dir || !dir[0]) return NULL;
  snprintf(path, sizeof(path), "%s/%s.csv", dir, test_id);
#endif
  path[sizeof(path) - 1] = '\0';

  fp = fopen(path, "w");
  if(!fp) return NULL;
  fprintf(fp, CSV_HEADER);
  fflush(fp);
  return fp;
}

static void
csv_row(
  FILE*       fp,
  const char* test_id,
  const char* sub_id,
  const char* solver,
  const char* diff_algo,
  double      px, double py, double pz,
  double      time_val,
  int         picard_order,
  int         nrealisations,
  double      E,
  double      SE,
  double      ref)
{
  double sigma_val;
  int pass_val;
  if(!fp) return;

  sigma_val = (SE > 1e-15) ? fabs(E - ref) / SE : 0.0;
  pass_val  = (SE > 1e-15) ? (sigma_val <= 3.0) : (fabs(E - ref) < 1e-10);

  fprintf(fp,
    "%s,%s,%s,%s,%.8g,%.8g,%.8g,%.8g,%d,%d,%.10g,%.6e,%.10g,%.4f,%d\n",
    test_id, sub_id, solver, diff_algo,
    px, py, pz, time_val,
    picard_order, nrealisations,
    E, SE, ref, sigma_val, pass_val);
  fflush(fp);
}

static void
csv_close(FILE* fp)
{
  if(fp) fclose(fp);
}

#endif /* TEST_SDIS_CSV_UTILS_H */
