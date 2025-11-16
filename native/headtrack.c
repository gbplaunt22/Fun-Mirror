#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
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

#define HEAD_WINDOW_SIZE 15
#define HEAD_WINDOW_HALF (HEAD_WINDOW_SIZE / 2)
#define MIN_WINDOW_VALID_PIXELS 45
#define MIN_WINDOW_MASK_FRACTION 0.35
#define MIN_CAP_DIFFERENCE_MM 8.0
#define MIN_VARIANCE_MM2 150.0

static freenect_context *f_ctx = NULL;
static freenect_device  *f_dev = NULL;
static uint16_t depth_buf[DW * DH];

static int headtrack_debug_windows_enabled(void)
{
    static int initialized = 0;
    static int enabled = 0;
    if (!initialized) {
        const char *env = getenv("HEADTRACK_DEBUG_WINDOWS");
        if (env && env[0]) {
            if (env[0] == '1' || env[0] == 't' || env[0] == 'T' || env[0] == 'y' || env[0] == 'Y') {
                enabled = 1;
            }
        }
        initialized = 1;
    }
    return enabled;
}

static void log_window_score(int u, int v, double coverage, double variance,
                             double cap_mm, double score, int passes)
{
    if (!headtrack_debug_windows_enabled())
        return;
    fprintf(stderr,
            "WINDOW_SCORE u=%d v=%d cov=%.2f var=%.1f cap=%.1f score=%.2f %s\n",
            u, v, coverage, variance, cap_mm, score, passes ? "PASS" : "FAIL");
}

// Kinect v1 approximate FOV (radians)
#define FOVX (57.0 * M_PI / 180.0)
#define FOVY (43.0 * M_PI / 180.0)

// -------- head finder on depth map --------

// Foreground pixels live within this many depth units of the closest sample we
// can reliably see.  The value is empirical but keeps us from pulling in random
// background scenery behind the player.
#define SLAB 400

// Approximate head width in meters, used to translate depth to a minimum
// horizontal blob span.  This lets us adapt the skinny-blob rejection logic to
// the player's distance from the camera so we keep working even when the player
// is further back.
#define HEAD_WIDTH_M 0.18

// Require at least this many pixels in a depth slab before we accept it as the
// "foreground" depth.  Without this, isolated noisy samples near the camera can
// shrink the slab to a tiny range and the tracker never finds the real player
// unless they stand extremely close.
#define MIN_FOREGROUND_PIXELS 150
#define MIN_LOCAL_SUPPORT 8

#define DEPTH_MIN_MM 300
#define DEPTH_MAX_MM 3500

static int estimate_min_blob_width(uint16_t depth_mm)
{
    // Avoid division by zero; clamp to a plausible range of operating depths.
    double z_m = depth_mm / 1000.0;
    if (z_m < 0.5)
        z_m = 0.5;
    if (z_m > 4.0)
        z_m = 4.0;

    double fx = DW / (2.0 * tan(FOVX / 2.0));
    double expected_px = HEAD_WIDTH_M * fx / z_m;

    // We only require part of the expected head width to be contiguous to allow
    // for occlusions or asymmetry, but still reject extremely skinny blobs.
    int min_width = (int)(expected_px * 0.35 + 0.5);
    if (min_width < 6)
        min_width = 6;
    if (min_width > (U_MAX - U_MIN))
        min_width = (U_MAX - U_MIN);
    return min_width;
}

static int slab_population(const int *prefix_hist, int start_depth)
{
    if (start_depth > DEPTH_MAX_MM)
        return 0;
    int end = start_depth + SLAB;
    if (end > DEPTH_MAX_MM)
        end = DEPTH_MAX_MM;
    int before = (start_depth > 0) ? prefix_hist[start_depth - 1] : 0;
    return prefix_hist[end] - before;
}

static int find_first_depth_in_slab(const int *depth_hist, int start_depth)
{
    if (start_depth > DEPTH_MAX_MM)
        return -1;
    int end = start_depth + SLAB;
    if (end > DEPTH_MAX_MM)
        end = DEPTH_MAX_MM;
    for (int d = start_depth; d <= end; ++d) {
        if (depth_hist[d] > 0)
            return d;
    }
    return -1;
}

static int select_head_window(const uint16_t *depth,
                              const uint8_t *mask,
                              const int *top_v,
                              int best_start,
                              int best_end,
                              int *out_u,
                              int *out_v,
                              uint16_t *out_z)
{
    int blob_top = V_MAX;
    for (int u = best_start; u <= best_end; ++u) {
        if (u < U_MIN || u >= U_MAX)
            continue;
        if (top_v[u] >= 0 && top_v[u] < blob_top) {
            blob_top = top_v[u];
        }
    }
    if (blob_top == V_MAX)
        return 0;

    int search_v_min = blob_top;
    if (search_v_min < V_MIN)
        search_v_min = V_MIN;
    int search_v_max = blob_top + HEAD_WINDOW_SIZE * 2;
    if (search_v_max >= V_MAX)
        search_v_max = V_MAX - 1;

    int u_min = best_start + HEAD_WINDOW_HALF;
    int u_max = best_end - HEAD_WINDOW_HALF;
    if (u_min < U_MIN + HEAD_WINDOW_HALF)
        u_min = U_MIN + HEAD_WINDOW_HALF;
    if (u_max > U_MAX - HEAD_WINDOW_HALF - 1)
        u_max = U_MAX - HEAD_WINDOW_HALF - 1;
    if (u_min > u_max)
        return 0;

    int v_min = search_v_min + HEAD_WINDOW_HALF;
    if (v_min < V_MIN + HEAD_WINDOW_HALF)
        v_min = V_MIN + HEAD_WINDOW_HALF;
    int v_max = search_v_max - HEAD_WINDOW_HALF;
    if (v_max > V_MAX - HEAD_WINDOW_HALF - 1)
        v_max = V_MAX - HEAD_WINDOW_HALF - 1;
    if (v_min > v_max)
        return 0;

    const int area = HEAD_WINDOW_SIZE * HEAD_WINDOW_SIZE;
    double best_score = -1e30;
    int found = 0;
    int best_u = 0;
    int best_v = 0;
    uint16_t best_z = 0;

    for (int vc = v_min; vc <= v_max; ++vc) {
        for (int uc = u_min; uc <= u_max; ++uc) {
            double sum = 0.0;
            double sum_sq = 0.0;
            int valid = 0;
            int mask_hits = 0;
            double border_sum = 0.0;
            int border_count = 0;
            for (int dv = -HEAD_WINDOW_HALF; dv <= HEAD_WINDOW_HALF; ++dv) {
                int row = (vc + dv) * DW;
                for (int du = -HEAD_WINDOW_HALF; du <= HEAD_WINDOW_HALF; ++du) {
                    int col = uc + du;
                    int idx = row + col;
                    if (mask[idx]) {
                        mask_hits++;
                    }
                    uint16_t d = depth[idx];
                    if (d == 0)
                        continue;
                    if (d < DEPTH_MIN_MM || d > DEPTH_MAX_MM)
                        continue;
                    double dval = (double)d;
                    sum += dval;
                    sum_sq += dval * dval;
                    valid++;
                    if (abs(du) == HEAD_WINDOW_HALF || abs(dv) == HEAD_WINDOW_HALF) {
                        border_sum += dval;
                        border_count++;
                    }
                }
            }
            if (valid < MIN_WINDOW_VALID_PIXELS)
                continue;
            if (border_count == 0)
                continue;
            double mean = sum / valid;
            double variance = (sum_sq / valid) - (mean * mean);
            if (variance < 0.0)
                variance = 0.0;
            double border_mean = border_sum / border_count;
            double cap_mm = border_mean - mean;
            double coverage = (double)mask_hits / (double)area;
            double score = cap_mm;
            if (variance > 0.0)
                score += sqrt(variance) * 0.25;
            score += coverage;

            int passes = 1;
            if (coverage < MIN_WINDOW_MASK_FRACTION)
                passes = 0;
            if (cap_mm < MIN_CAP_DIFFERENCE_MM)
                passes = 0;
            if (variance < MIN_VARIANCE_MM2)
                passes = 0;

            log_window_score(uc, vc, coverage, variance, cap_mm, score, passes);
            if (!passes)
                continue;
            if (!found || score > best_score) {
                found = 1;
                best_score = score;
                best_u = uc;
                best_v = vc;
                best_z = (uint16_t)(mean + 0.5);
            }
        }
    }

    if (!found)
        return 0;

    *out_u = best_u;
    *out_v = best_v;
    *out_z = best_z;
    return 1;
}

static int try_depth_candidate(const uint16_t *depth,
                               uint16_t best_depth,
                               int *out_u, int *out_v, uint16_t *out_z)
{

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
            if (d < DEPTH_MIN_MM)    continue;
            if (d > DEPTH_MAX_MM)     continue;
            if (d < best_depth) continue;
            if (d > slab_max)   continue;
            slab_mask[row + u] = 1;
        }
    }

    uint8_t eroded[DW * DH];
    memset(eroded, 0, sizeof(eroded));
    for (int v = V_MIN + 1; v < V_MAX - 1; ++v) {
        int row = v * DW;
        for (int u = U_MIN + 1; u < U_MAX - 1; ++u) {
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
    for (int v = V_MIN + 1; v < V_MAX - 1; ++v) {
        int row = v * DW;
        for (int u = U_MIN + 1; u < U_MAX - 1; ++u) {
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

    int min_blob_width = estimate_min_blob_width(best_depth);
    int max_blob_width = min_blob_width * 4;
    if (max_blob_width > (U_MAX - U_MIN))
        max_blob_width = (U_MAX - U_MIN);
    if (best_len < min_blob_width || best_len > max_blob_width || best_start < 0)
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

    int window_u = 0;
    int window_v = 0;
    uint16_t window_z = 0;
    if (select_head_window(depth, cleaned, top_v,
                           best_start, best_end,
                           &window_u, &window_v, &window_z)) {
        *out_u = window_u;
        *out_v = window_v;
        *out_z = window_z;
        return 1;
    }

    *out_u = centroid_u;
    *out_v = centroid_v;
    *out_z = best_depth;
    return 1;
}

static int attempt_depth_candidates(const uint16_t *depth,
                                    const int *depth_hist,
                                    const int *prefix_hist,
                                    int start_depth,
                                    int min_support,
                                    int *out_u, int *out_v, uint16_t *out_z)
{
    int s = start_depth;
    if (s < DEPTH_MIN_MM)
        s = DEPTH_MIN_MM;
    while (s <= DEPTH_MAX_MM) {
        int support = slab_population(prefix_hist, s);
        if (support >= min_support) {
            int best_depth = find_first_depth_in_slab(depth_hist, s);
            if (best_depth >= 0) {
                if (try_depth_candidate(depth, (uint16_t)best_depth,
                                        out_u, out_v, out_z)) {
                    return 1;
                }
                s = best_depth + SLAB;
                continue;
            }
        }
        s++;
    }
    return 0;
}

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
    uint16_t nearest_sample = 0xFFFF;
    int found_sample = 0;

    int depth_hist[DEPTH_MAX_MM + 1];
    memset(depth_hist, 0, sizeof(depth_hist));

    for (int v = V_MIN; v < V_MAX; ++v) {
        int row = v * DW;
        for (int u = U_MIN; u < U_MAX; ++u) {
            uint16_t d = depth[row + u];

            if (d == 0)      continue;     // invalid
            if (d < DEPTH_MIN_MM)    continue;     // too close
            if (d > DEPTH_MAX_MM)     continue;     // too far

            depth_hist[d]++;

            if (d < nearest_sample) {
                nearest_sample = d;
                found_sample = 1;
            }
        }
    }

    if (!found_sample)
        return 0;

    int prefix_hist[DEPTH_MAX_MM + 1];
    int accum = 0;
    for (int d = 0; d <= DEPTH_MAX_MM; ++d) {
        accum += depth_hist[d];
        prefix_hist[d] = accum;
    }

    int search_start = nearest_sample;
    if (search_start < DEPTH_MIN_MM)
        search_start = DEPTH_MIN_MM;

    int thresholds[] = {
        MIN_FOREGROUND_PIXELS,
        MIN_FOREGROUND_PIXELS / 2,
        MIN_FOREGROUND_PIXELS / 4,
        MIN_LOCAL_SUPPORT
    };
    const int num_thresholds = (int)(sizeof(thresholds) / sizeof(thresholds[0]));
    for (int i = 0; i < num_thresholds; ++i) {
        int min_support = thresholds[i];
        if (min_support < MIN_LOCAL_SUPPORT)
            min_support = MIN_LOCAL_SUPPORT;
        if (attempt_depth_candidates(depth, depth_hist, prefix_hist,
                                     search_start, min_support,
                                     out_u, out_v, out_z)) {
            return 1;
        }
    }

    return 0;
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
