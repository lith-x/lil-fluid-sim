#include <math.h>
#include <raylib.h>
#include <stdint.h>
#include <string.h>
#define ARENA_IMPLEMENTATION
#include "arena.h"

// ---- definitions ----

#define DISPLAY_WIDTH 512
#define DISPLAY_HEIGHT 512
#define MAX_FLOAT_VAL 20.0f
#define DIFF_COEFF 0.001f
#define VISC_COEFF 1.0f
#define GAUSS_ITERS 3
#define FORCE_SCALE 5.0f
#define COLOR_SCALE 50.0f

#define SIM_WIDTH (DISPLAY_WIDTH + 2)
#define SIM_HEIGHT (DISPLAY_HEIGHT + 2)
#define MEM_SIZE (SIM_WIDTH * SIM_HEIGHT)

#define G(arr, x, y) arr[((y) + 1) * SIM_WIDTH + (x) + 1]
#define swap_ptr(a, b)                                                         \
    do {                                                                       \
        void *tmp = a;                                                         \
        a = b;                                                                 \
        b = tmp;                                                               \
    } while (0);
#define add_source(arr, source, dt)                                            \
    do {                                                                       \
        for (int i = 0; i < MEM_SIZE; i++)                                     \
            arr[i] += dt * source[i];                                          \
    } while (0);

enum Dimension { DIM_SCALAR = 0, DIM_X = 1, DIM_Y = 2 };

// ---- util fns ----

void insert_rect(float *arr, int x, int y, int width, int height) {
    for (int lx = x; lx < x + width; lx++) {
        for (int ly = y; ly < y + height; ly++) {
            G(arr, lx, ly) = MAX_FLOAT_VAL;
        }
    }
}

void raw_to_display(float *floats, uint32_t *bytes) {
    for (int i = 0; i < MEM_SIZE; i++) {
        Color c = ColorFromHSV(remainderf((floats[i] * COLOR_SCALE), 360.0f),
                               1.0f, 1.0f);
        bytes[i] = *(uint32_t *)&c;
    }
}

void visualize_velocity(float *vel_x, float *vel_y) {
    for (int y = 0; y < DISPLAY_HEIGHT; y += 30) {
        for (int x = 0; x < DISPLAY_WIDTH; x += 30) {
            DrawCircle(x, y, 2, BLUE);
            DrawLine(x, y, x + G(vel_x, x, y), y + G(vel_y, x, y), BLUE);
        }
    }
}

// makes a spiral centered on the center of display
void setup_velocity(float *vel_x, float *vel_y) {
    float tan_scale = 1.0f;
    float rad_scale = -0.1f;
    float center_x = (float)DISPLAY_WIDTH / 2.0f;
    float center_y = (float)DISPLAY_HEIGHT / 2.0f;
    for (int y = 0; y < DISPLAY_HEIGHT; y++) {
        for (int x = 0; x < DISPLAY_WIDTH; x++) {
            float dx = x - center_x;
            float dy = y - center_y;

            // Tangential component (rotational)
            float vx_tan = -dy;
            float vy_tan = dx;

            // Radial component (inward/outward)
            float vx_rad = dx;
            float vy_rad = dy;

            // Combine
            G(vel_x, x, y) =
                FORCE_SCALE * (tan_scale * vx_tan + rad_scale * vx_rad);
            G(vel_y, x, y) =
                FORCE_SCALE * (tan_scale * vy_tan + rad_scale * vy_rad);
        }
    }
}

// ---- sim fns ----

// there might be something wrong here, left/top boundary replicates/produces
// neighbor, right/bottom holds sources beyond what they're supposed to. could
// also be a symptom of the current static velocity field that may get sorted
// out once it becomes dynamic (field has overall negative divergence)
void reset_boundary(float *arr, int dim) {
    // Set boundary negative so it "cancels out" on edges, i think?
    if (dim == DIM_X) {
        for (int y = 0; y < DISPLAY_HEIGHT; y++) {
            G(arr, -1, y) = -G(arr, 0, y);
            G(arr, DISPLAY_WIDTH + 1, y) = -G(arr, DISPLAY_WIDTH, y);
        }
    } else {
        for (int y = 0; y < DISPLAY_HEIGHT; y++) {
            G(arr, 0, y) = G(arr, 1, y);
            G(arr, DISPLAY_WIDTH + 1, y) = G(arr, DISPLAY_WIDTH, y);
        }
    }

    if (dim == DIM_Y) {
        for (int y = 0; y < DISPLAY_HEIGHT; y++) {
            G(arr, -1, y) = -G(arr, 0, y);
            G(arr, DISPLAY_WIDTH + 1, y) = -G(arr, DISPLAY_WIDTH, y);
        }
    } else {
        for (int y = 0; y < DISPLAY_HEIGHT; y++) {
            G(arr, 0, y) = G(arr, 1, y);
            G(arr, DISPLAY_WIDTH + 1, y) = G(arr, DISPLAY_WIDTH, y);
        }
    }

    for (int x = 0; x < DISPLAY_WIDTH; x++) {
        if (dim == DIM_Y) {
            G(arr, x, -1) = -G(arr, x, 0);
            G(arr, x, DISPLAY_HEIGHT + 1) = -G(arr, x, DISPLAY_HEIGHT);
        } else {
            G(arr, x, -1) = G(arr, x, 0);
            G(arr, x, DISPLAY_HEIGHT + 1) = G(arr, x, DISPLAY_HEIGHT);
        }
    }

    G(arr, -1, -1) = 0.5f * (G(arr, -1, 0) + G(arr, 0, -1));
    G(arr, -1, DISPLAY_HEIGHT + 1) =
        0.5f * (G(arr, 0, DISPLAY_HEIGHT + 1) + G(arr, -1, DISPLAY_HEIGHT));
    G(arr, DISPLAY_WIDTH + 1, -1) =
        0.5f * (G(arr, DISPLAY_WIDTH + 1, 0) + G(arr, DISPLAY_WIDTH, -1));
    G(arr, DISPLAY_WIDTH + 1, DISPLAY_HEIGHT + 1) =
        0.5f * (G(arr, DISPLAY_WIDTH + 1, DISPLAY_HEIGHT) +
                G(arr, DISPLAY_WIDTH, DISPLAY_HEIGHT + 1));
}

void diffuse(int dim, float *p, float *n, float coeff, float dt) {
    float diff_scale = coeff * dt;
    float a_x = diff_scale * (float)(DISPLAY_WIDTH * DISPLAY_WIDTH);
    float a_y = diff_scale * (float)(DISPLAY_HEIGHT * DISPLAY_HEIGHT);
    float denom = 1.0f / (1.0f + 2.0f * (a_x + a_y));

    for (int k = 0; k < GAUSS_ITERS; k++) {
        for (int y = 0; y < DISPLAY_HEIGHT; y++) {
            for (int x = 0; x < DISPLAY_WIDTH; x++) {
                G(n, x, y) = denom * (G(p, x, y) +
                                      a_x * (G(n, x + 1, y) + G(n, x - 1, y)) +
                                      a_y * (G(n, x, y + 1) + G(n, x, y - 1)));
            }
        }
    }
    reset_boundary(n, dim);
}

void advect(int dim, float *p, float *n, float *vel_x, float *vel_y, float dt) {
    float dtx = dt; // * DISPLAY_WIDTH;
    float dty = dt; // * DISPLAY_HEIGHT;
    for (int y = 0; y < DISPLAY_HEIGHT; y++) {
        for (int x = 0; x < DISPLAY_WIDTH; x++) {
            float traced_x = fminf(DISPLAY_WIDTH + 0.5f,
                                   fmaxf(0.5f, x - dtx * G(vel_x, x, y)));
            float traced_y = fminf(DISPLAY_HEIGHT + 0.5f,
                                   fmaxf(0.5f, y - dty * G(vel_y, x, y)));
            int x0 = (int)traced_x;
            int x1 = x0 + 1;
            int y0 = (int)traced_y;
            int y1 = y0 + 1;

            x0 = fmaxf(0, fminf(x0, DISPLAY_WIDTH - 1));
            x1 = fmaxf(0, fminf(x1, DISPLAY_WIDTH - 1));
            y0 = fmaxf(0, fminf(y0, DISPLAY_HEIGHT - 1));
            y1 = fmaxf(0, fminf(y1, DISPLAY_HEIGHT - 1));

            float s1 = traced_x - x0;
            float s0 = 1.0f - s1;
            float t1 = traced_y - y0;
            float t0 = 1.0f - t1;
            G(n, x, y) = s0 * (t0 * G(p, x0, y0) + t1 * G(p, x0, y1)) +
                         s1 * (t0 * G(p, x1, y0) + t1 * G(p, x1, y1));
        }
    }
    reset_boundary(n, dim);
}

void project(float *vel_x, float *vel_y, float *poisson, float *divergence) {
    float h_x = (float)DISPLAY_WIDTH;
    float h_x2 = h_x * h_x;
    float h_y = (float)DISPLAY_HEIGHT;
    float h_y2 = h_y * h_y;

    for (int y = 0; y < DISPLAY_HEIGHT; y++) {
        for (int x = 0; x < DISPLAY_WIDTH; x++) {
            G(divergence, x, y) =
                -((G(vel_x, x + 1, y) - G(vel_x, x - 1, y)) / (2.0f * h_x) +
                  (G(vel_y, x, y + 1) - G(vel_y, x, y - 1)) / (2.0f * h_y));
            G(poisson, x, y) = 0.0f;
        }
    }
    reset_boundary(divergence, DIM_SCALAR);
    reset_boundary(poisson, DIM_SCALAR);

    for (int k = 0; k < GAUSS_ITERS; k++) {
        for (int y = 0; y < DISPLAY_HEIGHT; y++) {
            for (int x = 0; x < DISPLAY_WIDTH; x++) {
                G(poisson, x, y) =
                    ((G(poisson, x - 1, y) + G(poisson, x + 1, y)) * h_y2 +
                     (G(poisson, x, y - 1) + G(poisson, x, y + 1)) * h_x2 -
                     G(divergence, x, y) * h_x2 * h_y2) /
                    (2.0f * (h_x2 + h_y2));
            }
        }
        reset_boundary(poisson, DIM_SCALAR);
    }

    for (int y = 0; y < DISPLAY_HEIGHT; y++) {
        for (int x = 0; x < DISPLAY_WIDTH; x++) {
            G(vel_x, x, y) -=
                (G(poisson, x + 1, y) - G(poisson, x - 1, y)) / (2.0f * h_x);
            G(vel_y, x, y) -=
                (G(poisson, x, y + 1) - G(poisson, x, y - 1)) / (2.0f * h_y);
        }
    }
    reset_boundary(vel_x, DIM_X);
    reset_boundary(vel_y, DIM_Y);
}

// ---- main ----

int main() {
    Arena arena = {0};
    uint32_t *displayed = arena_alloc(&arena, MEM_SIZE * sizeof(uint32_t));

    float *dens = arena_alloc(&arena, MEM_SIZE * sizeof(float));
    float *dens0 = arena_alloc(&arena, MEM_SIZE * sizeof(float));

    float *vel_x = arena_alloc(&arena, MEM_SIZE * sizeof(float));
    float *vel_x0 = arena_alloc(&arena, MEM_SIZE * sizeof(float));
    float *vel_y = arena_alloc(&arena, MEM_SIZE * sizeof(float));
    float *vel_y0 = arena_alloc(&arena, MEM_SIZE * sizeof(float));
    // setup_velocity(vel_x, vel_y);

    // insert_rect(dens1, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    // float_to_byte(dens1, displayed);

    Image image = {.data = displayed,
                   .format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8,
                   .height = SIM_HEIGHT,
                   .width = SIM_WIDTH,
                   .mipmaps = 1};

    InitWindow(DISPLAY_WIDTH, DISPLAY_HEIGHT, "hi");
    Texture2D tex = LoadTextureFromImage(image);
    SetTargetFPS(GetMonitorRefreshRate(GetCurrentMonitor()));

    float dt;
    int mouse_x, mouse_y;
    Vector2 mouse_delta;
    char mouse_text[128];
    while (!WindowShouldClose()) {
        dt = GetFrameTime();
        mouse_x = GetMouseX();
        mouse_y = GetMouseY();
        mouse_delta = GetMouseDelta();
        // add sources
        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
            for (int y = mouse_y - 5; y < mouse_y + 5; y++) {
                for (int x = mouse_x - 5; x < mouse_x + 5; x++) {
                    G(dens0, (int)fminf(DISPLAY_WIDTH, fmaxf(0, x)),
                      (int)fminf(DISPLAY_HEIGHT, fmaxf(0, y))) =
                        MAX_FLOAT_VAL * 50.0f;
                    G(vel_x, x, y) += fmaxf(-10.0f, fminf(10.0f, mouse_delta.x * 0.5f));
                    G(vel_y, x, y) += fmaxf(-10.0f, fminf(10.0f, mouse_delta.y * 0.5f));
                }
            }
        }
        // memset(vel_x0, 0, MEM_SIZE * sizeof(float));
        // memset(vel_y0, 0, MEM_SIZE * sizeof(float));
        // velocity step
        add_source(vel_x, vel_x0, dt);
        add_source(vel_y, vel_y0, dt);
        swap_ptr(vel_x, vel_x0);
        diffuse(DIM_X, vel_x0, vel_x, VISC_COEFF, dt);
        swap_ptr(vel_y, vel_y0);
        diffuse(DIM_Y, vel_y0, vel_y, VISC_COEFF, dt);
        project(vel_x, vel_y, vel_x0, vel_y0);
        swap_ptr(vel_x, vel_x0);
        swap_ptr(vel_y, vel_y0);
        advect(DIM_X, vel_x, vel_x0, vel_x0, vel_y0, dt);
        advect(DIM_Y, vel_y, vel_y0, vel_x0, vel_y0, dt);
        project(vel_x, vel_y, vel_x0, vel_y0);

        // density step
        add_source(dens, dens0, dt);
        swap_ptr(dens, dens0);
        diffuse(DIM_SCALAR, dens, dens0, DIFF_COEFF, dt);
        swap_ptr(dens, dens0);
        advect(DIM_SCALAR, dens, dens0, vel_x, vel_y, dt);

        raw_to_display(dens, displayed);
        UpdateTexture(tex, displayed);

        snprintf(mouse_text, 128, "(%d, %d) : %.03f\n(%.02f, %.02f)\n%.02f",
                 mouse_x, mouse_y, G(dens, mouse_x, mouse_y),
                 G(vel_x, mouse_x, mouse_y), G(vel_y, mouse_x, mouse_y), dt);

        BeginDrawing();
        DrawTexture(tex, -1, -1, WHITE);
        visualize_velocity(vel_x, vel_y);
        DrawRectangle(0, 0, DISPLAY_WIDTH / 2, 75, BLACK);
        DrawText(mouse_text, 0, 0, 16, RED);
        EndDrawing();
    }
    CloseWindow();
    arena_free(&arena);
    return 0;
}