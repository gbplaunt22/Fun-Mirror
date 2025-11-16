#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <libfreenect.h>

#define DW 640
#define DH 480

#define GRID_W 40
#define GRID_H 20

static freenect_context *f_ctx = NULL;
static freenect_device  *f_dev = NULL;
static uint16_t depth_buf[DW * DH];

// Kinect v1 approximate FOV (radians)
#define FOVX (57.0 * M_PI / 180.0)
#define FOVY (43.0 * M_PI / 180.0)

// -------- simple head finder on depth map --------

// Find a "head" pixel in the central region by taking the CLOSEST valid pixel.
// Returns 1 if found, 0 otherwise.
static int find_head_pixel(const uint16_t *depth,
                           int *out_u, int *out_v, uint16_t *out_z)
{
    // Central region (tunable)
    int u_min = DW * 3 / 10;
    int u_max = DW * 7 / 10;
    int v_min = DH * 2 / 10;
    int v_max = DH * 8 / 10;

    // 1) Find minimum valid depth in this region
    uint16_t d_min = 0xFFFF;
    for (int v = v_min; v < v_max; ++v) {
        int row = v * DW;
        for (int u = u_min; u < u_max; ++u) {
            uint16_t d = depth[row + u];
            if (d == 0) continue;          // invalid
            if (d < 200 || d > 4000) continue; // ignore absurd near/far
            if (d < d_min)
                d_min = d;
        }
    }
    if (d_min == 0xFFFF) {
        // nothing plausible
        return 0;
    }

    // 2) Foreground slab: a band in front of the camera around the user
    const uint16_t SLAB = 400; // ~40cm thickness; tweak if needed
    uint16_t near = d_min;
    uint16_t far  = d_min + SLAB;

    // 3) For each column in the band, find the top-most foreground pixel
    //    top_v[u] = first (smallest v) where depth in [near, far], or -1
    static int top_v[DW];
    for (int u = 0; u < DW; ++u)
        top_v[u] = -1;

    for (int v = v_min; v < v_max; ++v) {
        int row = v * DW;
        for (int u = u_min; u < u_max; ++u) {
            uint16_t d = depth[row + u];
            if (d == 0) continue;
            if (d < near || d > far) continue;

            if (top_v[u] == -1) {
                top_v[u] = v;  // first hit in this column = top-most
            }
        }
    }

    // 4) Aggregate columns to get a stable head point
    int sum_u = 0;
    int count_u = 0;
    int best_v = DH;  // smallest v among contributing columns

    for (int u = u_min; u < u_max; ++u) {
        if (top_v[u] != -1) {
            sum_u += u;
            count_u++;
            if (top_v[u] < best_v)
                best_v = top_v[u];
        }
    }

    if (count_u == 0)
        return 0;

    int u_head = sum_u / count_u;
    int v_head = best_v;

    // Get an actual depth value near the head location
    uint16_t z_head = depth[v_head * DW + u_head];
    if (z_head == 0) {
        // fall back to d_min if this particular pixel is invalid
        z_head = d_min;
    }

    *out_u = u_head;
    *out_v = v_head;
    *out_z = z_head;
    return 1;
}
    // Depth thresholds in raw 11-bit units.
    const uint16_t NEAR = 200;   // ignore stuff too close
    const uint16_t FAR  = 2000;  // ignore stuff too far

    // Horizontal band: middle 40% of the image
    int u_min = DW * 3 / 10;
    int u_max = DW * 7 / 10;

    // Vertical band: only the “mirror area”
    int v_min = DH * 2 / 10;
    int v_max = DH * 8 / 10;

    uint16_t best_depth = 0xFFFF;
    int best_u = -1;
    int best_v = -1;

    int v, u;
    for (v = v_min; v < v_max; ++v) {
        int row = v * DW;
        for (u = u_min; u < u_max; ++u) {
            uint16_t d = depth[row + u];
            if (d == 0)      continue;      // invalid
            if (d < NEAR)    continue;      // too close
            if (d > FAR)     continue;      // too far

            // Choose the CLOSEST pixel in this band
            if (d < best_depth) {
                best_depth = d;
                best_u = u;
                best_v = v;
            }
        }
    }

    if (best_u >= 0) {
        *out_u = best_u;
        *out_v = best_v;
        *out_z = best_depth;
        return 1;
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

    // We don't need spam every frame, so only visualize every 5th frame
    if (frame_count % 5 != 0)
        return;

    int u_head, v_head;
    uint16_t z_head;
    if (!find_head_pixel(depth_buf, &u_head, &v_head, &z_head)) {
        // No plausible head found; just print a note occasionally
        return;
    }

    double hx, hy, hz;
    head_pixel_to_3d(u_head, v_head, z_head, &hx, &hy, &hz);

     // --- simple temporal smoothing (exponential moving average) ---
    static int have_prev = 0;
    static double shx = 0.0, shy = 0.0, shz = 0.0;
    const double alpha = 0.25;  // 0..1; higher = more responsive, lower = smoother

    if (have_prev) {
        hx = alpha * hx + (1.0 - alpha) * shx;
        hy = alpha * hy + (1.0 - alpha) * shy;
        hz = alpha * hz + (1.0 - alpha) * shz;
    } else {
        have_prev = 1;
    }

    shx = hx;
    shy = hy;
    shz = hz;
    // --------------------------------------------------------------

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
