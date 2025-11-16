#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <libfreenect.h>

#define DW 640
#define DH 480

// Search window (relative to depth frame) that find_head_pixel() scans.
// Keeping the window centralized reduces noise, but the coordinates we emit
// need to be normalized to the window instead of the full sensor range or the
// UI barely moves.  Keep the bounds in one place so both the detector and the
// exporter agree.
#define U_MIN (DW * 3 / 10)
#define U_MAX (DW * 7 / 10)
#define V_MIN (DH * 2 / 10)
#define V_MAX (DH * 8 / 10)

static freenect_context *f_ctx = NULL;
static freenect_device  *f_dev = NULL;
static uint16_t depth_buf[DW * DH];

// Kinect v1 approximate FOV (radians)
#define FOVX (57.0 * M_PI / 180.0)
#define FOVY (43.0 * M_PI / 180.0)

// -------- head finder on depth map --------

// Foreground mask parameters for blob selection
#define SLAB_MM            250   // pixels within best_depth .. best_depth+SLAB are kept
#define MIN_COL_PIXELS      12   // reject skinny columns that are just noise
#define MIN_RUN_COLUMNS      6   // need this many adjacent support columns

// Find a "head" pixel using a depth-based person blob:
//  1) find nearest valid depth in the central window
//  2) treat pixels within [d_min, d_min + SLAB] as foreground
//  3) erode/dilate the mask to discard speckles
//  4) take the top-most pixels across the dominant column run as the head point
// Returns 1 if found, 0 otherwise.
static int find_head_pixel(const uint16_t *depth,
                           int *out_u, int *out_v, uint16_t *out_z)
{
    // Throw away crazy distances
    const uint16_t NEAR = 300;   // too close (< ~0.5m) – ignore
    const uint16_t FAR  = 3500;  // too far  (> ~3.5m) – ignore

    uint16_t best_depth = 0xFFFF;
    int best_u = -1;
    int best_v = -1;

    for (int v = V_MIN; v < V_MAX; ++v) {
        int row = v * DW;
        for (int u = U_MIN; u < U_MAX; ++u) {
            uint16_t d = depth[row + u];

            if (d == 0)      continue;     // invalid
            if (d < NEAR)    continue;     // too close
            if (d > FAR)     continue;     // too far

            if (d < best_depth) {
                best_depth = d;
                best_u = u;
                best_v = v;
            }
        }
    }

    if (best_u < 0)
        return 0;

    // Build a foreground mask inside the search window for pixels within the
    // same depth slab as the best sample.
    const int region_w = U_MAX - U_MIN;
    const int region_h = V_MAX - V_MIN;
    const uint16_t slab_max = best_depth + SLAB_MM;

    uint8_t mask[region_w * region_h];
    memset(mask, 0, sizeof(mask));

    for (int v = 0; v < region_h; ++v) {
        int row = (V_MIN + v) * DW;
        for (int u = 0; u < region_w; ++u) {
            uint16_t d = depth[row + (U_MIN + u)];
            if (d == 0) continue;
            if (d < NEAR || d > FAR) continue;
            if (d >= best_depth && d <= slab_max) {
                mask[v * region_w + u] = 1;
            }
        }
    }

    // Quick morphological cleanup: 3x3 erosion followed by dilation.  This
    // helps reject isolated pixels without pulling in the entire background.
    uint8_t eroded[region_w * region_h];
    memset(eroded, 0, sizeof(eroded));
    for (int v = 1; v < region_h - 1; ++v) {
        for (int u = 1; u < region_w - 1; ++u) {
            int idx = v * region_w + u;
            int sum = 0;
            for (int dv = -1; dv <= 1; ++dv) {
                for (int du = -1; du <= 1; ++du) {
                    sum += mask[(v + dv) * region_w + (u + du)];
                }
            }
            if (sum == 9)
                eroded[idx] = 1;
        }
    }

    uint8_t cleaned[region_w * region_h];
    memset(cleaned, 0, sizeof(cleaned));
    for (int v = 1; v < region_h - 1; ++v) {
        for (int u = 1; u < region_w - 1; ++u) {
            int idx = v * region_w + u;
            int sum = 0;
            for (int dv = -1; dv <= 1; ++dv) {
                for (int du = -1; du <= 1; ++du) {
                    sum += eroded[(v + dv) * region_w + (u + du)];
                }
            }
            if (sum > 0)
                cleaned[idx] = 1;
        }
    }

    int column_top[region_w];
    int column_count[region_w];
    for (int u = 0; u < region_w; ++u) {
        column_top[u] = -1;
        column_count[u] = 0;
        for (int v = 0; v < region_h; ++v) {
            if (cleaned[v * region_w + u]) {
                column_count[u]++;
                if (column_top[u] < 0)
                    column_top[u] = v;
            }
        }
    }

    // Find the widest contiguous run of well-supported columns.  This tends to
    // latch onto the top of the largest blob (the person) instead of stray
    // hands.
    int best_run_start = -1;
    int best_run_end = -1;
    int best_run_score = 0;
    int run_start = -1;
    int run_score = 0;
    int run_len = 0;
    for (int u = 0; u < region_w; ++u) {
        if (column_top[u] >= 0 && column_count[u] >= MIN_COL_PIXELS) {
            if (run_start < 0) {
                run_start = u;
                run_score = column_count[u];
                run_len = 1;
            } else {
                run_score += column_count[u];
                run_len++;
            }
        } else {
            if (run_len >= MIN_RUN_COLUMNS && run_score > best_run_score) {
                best_run_score = run_score;
                best_run_start = run_start;
                best_run_end = run_start + run_len - 1;
            }
            run_start = -1;
            run_score = 0;
            run_len = 0;
        }
    }
    if (run_len >= MIN_RUN_COLUMNS && run_score > best_run_score) {
        best_run_score = run_score;
        best_run_start = run_start;
        best_run_end = run_start + run_len - 1;
    }

    if (best_run_start >= 0) {
        double sum_u = 0.0;
        double sum_v = 0.0;
        int samples = 0;
        for (int u = best_run_start; u <= best_run_end; ++u) {
            int top = column_top[u];
            if (top < 0)
                continue;
            sum_u += (U_MIN + u);
            sum_v += (V_MIN + top);
            samples++;
        }
        if (samples > 0) {
            *out_u = (int)lrint(sum_u / samples);
            *out_v = (int)lrint(sum_v / samples);
            *out_z = best_depth;
            return 1;
        }
    }

    // Fall back to the single best pixel if we could not form a stable blob.
    *out_u = best_u;
    *out_v = best_v;
    *out_z = best_depth;
    return 1;
}



// Convert head pixel (u,v,z) into rough 3D coordinates in Kinect space (meters)
static void head_pixel_to_3d(int u, int v, uint16_t z_raw,
                             double *x, double *y, double *z)
{
    double z_m = z_raw / 1000.0;   // fake mm -> m scale for 11-bit depth

    double cx = DW / 2.0;
    double cy = DH / 2.0;
    double fx = DW / (2.0 * tan(FOVX / 2.0));
    double fy = DH / (2.0 * tan(FOVY / 2.0));

    *x = (u - cx) * z_m / fx;
    *y = (v - cy) * z_m / fy;
    *z = z_m;
}

// -------- libfreenect depth callback --------

static void depth_cb(freenect_device *dev, void *v_depth, uint32_t ts)
{
    uint16_t *src = (uint16_t *)v_depth;
    memcpy(depth_buf, src, sizeof(depth_buf));

    static int frame_count = 0;
    frame_count++;

    // only every 5th frame so we don't spam
    if (frame_count % 5 != 0)
        return;

    int u_head, v_head;
    uint16_t z_head;
    if (!find_head_pixel(depth_buf, &u_head, &v_head, &z_head)) {
        return;
    }

    // --- Normalized coords in [-1, 1] relative to the search window ---
    double hx = ( (double)(u_head - U_MIN) / (double)(U_MAX - U_MIN) ) * 2.0 - 1.0;
    double hy = ( (double)(v_head - V_MIN) / (double)(V_MAX - V_MIN) ) * 2.0 - 1.0;
    if (hx < -1.0) hx = -1.0;
    if (hx >  1.0) hx =  1.0;
    if (hy < -1.0) hy = -1.0;
    if (hy >  1.0) hy =  1.0;
    double hz = z_head / 1000.0;                   // meters-ish

    // Also keep the sensor-relative values for debugging so we can see what
    // the detector actually latched on to.
    double hx_raw = (u_head - DW / 2.0) / (DW / 2.0);
    double hy_raw = (v_head - DH / 2.0) / (DH / 2.0);

    // Helpful debug to stderr: where in the depth image is this?
    fprintf(stderr,
            "HEAD_PIXEL u=%d v=%d z=%u  -> raw(%.3f, %.3f) window-norm(%.3f, %.3f) hz=%.3f\n",
            u_head, v_head, z_head, hx_raw, hy_raw, hx, hy, hz);

    // This is what Java parses: RAW values only
    printf("HEAD %.6f %.6f %.6f\n", hx, hy, hz);
    fflush(stdout);
}


// -------- main program --------

int main(void)
{
    if (freenect_init(&f_ctx, NULL) < 0) {
        fprintf(stderr, "freenect_init failed\n");
        return 1;
    }

    freenect_select_subdevices(f_ctx, FREENECT_DEVICE_CAMERA);

    if (freenect_open_device(f_ctx, &f_dev, 0) < 0) {
        fprintf(stderr, "could not open device 0\n");
        freenect_shutdown(f_ctx);
        return 1;
    }

    freenect_set_depth_callback(f_dev, depth_cb);
    freenect_set_depth_mode(
        f_dev,
        freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM,
                                 FREENECT_DEPTH_11BIT)
    );

    if (freenect_start_depth(f_dev) < 0) {
        fprintf(stderr, "could not start depth\n");
        freenect_close_device(f_dev);
        freenect_shutdown(f_ctx);
        return 1;
    }

    fprintf(stderr,
            "headtrack: running. Stand ~1.5–3m in front of Kinect; Ctrl+C to quit.\n");

    while (1) {
        int rc = freenect_process_events(f_ctx);
        if (rc < 0) {
            fprintf(stderr, "freenect_process_events error: %d\n", rc);
            break;
        }
    }

    freenect_stop_depth(f_dev);
    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);

    return 0;
}
