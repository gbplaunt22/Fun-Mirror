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

    // Print numeric info under the grid for parsing
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
