// Reads per-sample dx in mm from stdin, runs the firmware's detector and
// corrector, and writes corrected dx in mm to stdout.
// argv[1] = "forced": full correction whenever tremor-band energy is present
//           (matches the Python method comparison)
// argv[1] = "real":   the firmware's own tremor flag and intensity-based depth
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "tremor_detect.h"
#include "correction.h"

int main(int argc, char **argv) {
    int forced = argc > 1 && strcmp(argv[1], "forced") == 0;
    detector_init();
    corrector_init();
    tremor_params_t p = {0};
    int since = 0;
    double dx, ax;
    while (scanf("%lf %lf", &dx, &ax) == 2) {
        detector_push((float)dx, 0.0f, (float)ax, 0.0f);
        if (++since >= ANALYSIS_EVERY_N) { since = 0; detector_analyze(&p); }
        tremor_params_t q = p;
        if (forced && q.seq) { q.tremor = true; q.intensity_mm = 100.0f; }   // depth -> 1
        corrector_set_params(&q, false);
        hid_delta_t o = corrector_process((float)(dx / COUNTS_TO_MM), 0.0f);
        printf("%.9f\n", o.dx * COUNTS_TO_MM);
    }
    return 0;
}
