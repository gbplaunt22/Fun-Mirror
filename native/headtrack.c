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

// Foreground pixels live within this many depth units of the closest sample we
// can reliably see.  The value is empirical but keeps us from pulling in random
// background scenery behind the player.
#define SLAB 400

// Reject skinny blobs that are narrower than this many columns.  Again, this is
// an empirical choice but avoids locking on to noise.
#define MIN_BLOB_WIDTH 20

// Find a "head" pixel using a depth-based person blob:
//  1) find nearest valid depth in a central region
//  2) treat pixels within [d_min, d_min + SLAB] as foreground
//  3) clean the mask with a 3x3 erosion/dilation to suppress speckles
//  4) scan columns to find the largest contiguous blob of top-most pixels
//  5) return the centroid of those top pixels for a stable head point
// Find a "head" pixel in the central region by taking the CLOSEST valid pixel.
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

    // Foreground mask of everything near the closest observed depth.
    uint8_t slab_mask[DW * DH];
    memset(slab_mask, 0, sizeof(slab_mask));
    uint16_t slab_max = best_depth + SLAB;
    if (slab_max < best_depth) {
        slab_max = 0xFFFF;  // overflow guard
    }

    for (int v = V_MIN; v < V_MAX; ++v) {
        int row = v * DW;
        for (int u = U_MIN; u < U_MAX; ++u) {
            uint16_t d = depth[row + u];
            if (d == 0)      continue;
            if (d < NEAR)    continue;
            if (d > FAR)     continue;
            if (d < best_depth) continue;
            if (d > slab_max)   continue;
            slab_mask[row + u] = 1;
        }
    }

    // Morphological clean-up: 3x3 erosion followed by dilation.  This keeps
    // the implementation portable (no SIMD assumptions) while removing speckle
    // noise from the mask.
    uint8_t eroded[DW * DH];
    memset(eroded, 0, sizeof(eroded));
    for (int v = 1; v < DH - 1; ++v) {
        int row = v * DW;
        for (int u = 1; u < DW - 1; ++u) {
            if (!slab_mask[row + u])
                continue;
            int all_neighbors = 1;
            for (int dv = -1; dv <= 1 && all_neighbors; ++dv) {
                int nrow = (v + dv) * DW;
                for (int du = -1; du <= 1; ++du) {
                    if (!slab_mask[nrow + (u + du)]) {
                        all_neighbors = 0;
                        break;
                    }
                }
            }
            if (all_neighbors) {
                eroded[row + u] = 1;
            }
        }
    }

    uint8_t cleaned[DW * DH];
    memset(cleaned, 0, sizeof(cleaned));
    for (int v = 1; v < DH - 1; ++v) {
        int row = v * DW;
        for (int u = 1; u < DW - 1; ++u) {
            int any_neighbors = 0;
            for (int dv = -1; dv <= 1 && !any_neighbors; ++dv) {
                int nrow = (v + dv) * DW;
                for (int du = -1; du <= 1; ++du) {
                    if (eroded[nrow + (u + du)]) {
                        any_neighbors = 1;
                        break;
                    }
                }
            }
            if (any_neighbors) {
                cleaned[row + u] = 1;
            }
        }
    }

    int top_v[DW];
    for (int u = 0; u < DW; ++u) {
        top_v[u] = -1;
    }

    int total_columns = 0;
    for (int u = U_MIN; u < U_MAX; ++u) {
        for (int v = V_MIN; v < V_MAX; ++v) {
            if (cleaned[v * DW + u]) {
                top_v[u] = v;
                total_columns++;
                break;
            }
        }
    }

    if (total_columns == 0)
        return 0;

    // Locate the widest contiguous run of columns that have a valid top pixel.
    int best_start = -1;
    int best_end = -1;
    int best_len = 0;
    int cur_start = -1;
    int cur_len = 0;
    for (int u = U_MIN; u < U_MAX; ++u) {
        if (top_v[u] >= 0) {
            if (cur_len == 0) {
                cur_start = u;
            }
            cur_len++;
        } else if (cur_len > 0) {
            if (cur_len > best_len) {
                best_len = cur_len;
                best_start = cur_start;
                best_end = u - 1;
            }
            cur_len = 0;
        }
    }
    if (cur_len > best_len) {
        best_len = cur_len;
        best_start = cur_start;
        best_end = U_MAX - 1;
    }

    if (best_len < MIN_BLOB_WIDTH || best_start < 0)
        return 0;

    double sum_u = 0.0;
    double sum_v = 0.0;
    int count = 0;
    for (int u = best_start; u <= best_end; ++u) {
        if (top_v[u] >= 0) {
            sum_u += u;
            sum_v += top_v[u];
            count++;
        }
    }

    if (count == 0)
        return 0;

    double mean_u = sum_u / count;
    double mean_v = sum_v / count;
    int centroid_u = (int)(mean_u >= 0.0 ? mean_u + 0.5 : mean_u - 0.5);
    int centroid_v = (int)(mean_v >= 0.0 ? mean_v + 0.5 : mean_v - 0.5);

    *out_u = centroid_u;
    *out_v = centroid_v;
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
