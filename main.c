#include <math.h>
#include <raylib.h>
#include <stdint.h>
#define ARENA_IMPLEMENTATION
#include "arena.h"

#define DISPLAY_WIDTH 512
#define DISPLAY_HEIGHT 512
#define MAX_FLOAT_VAL 20.0f
#define DIFF_COEFF 20.0f
#define GAUSS_ITERS 10
#define FORCE_SCALE 1.0f

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

void insert_rect(float *arr, int x, int y, int width, int height) {
    for (int lx = x; lx < x + width; lx++) {
        for (int ly = y; ly < y + height; ly++) {
            G(arr, lx, ly) = MAX_FLOAT_VAL;
        }
    }
}

void float_to_byte(float *floats, uint8_t *bytes) {
    for (int i = 0; i < MEM_SIZE; i++)
        bytes[i] = (uint8_t)((floats[i] / MAX_FLOAT_VAL) * (float)UINT8_MAX);
}

void reset_boundary(float *arr) {
    // Set boundary negative so it "cancels out" on edges, i think?
    for (int x = 0; x < DISPLAY_WIDTH; x++) {
        G(arr, x, -1) = -G(arr, x, -1);
        G(arr, x, DISPLAY_HEIGHT + 1) = -G(arr, x, DISPLAY_HEIGHT + 1);
    }
    for (int y = 0; y < DISPLAY_HEIGHT; y++) {
        G(arr, -1, y) = -G(arr, -1, y);
        G(arr, DISPLAY_WIDTH + 1, y) = -G(arr, DISPLAY_WIDTH + 1, y);
    }
    // Corners are a weird case, just set to 0
    G(arr, DISPLAY_WIDTH + 1, DISPLAY_HEIGHT + 1) = 0;
    G(arr, DISPLAY_WIDTH + 1, -1) = 0;
    G(arr, -1, DISPLAY_HEIGHT + 1) = 0;
    G(arr, -1, -1) = 0;
}

void diffuse(float *p, float *n, float dt) {
    float diff_scale = DIFF_COEFF * dt;
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
    reset_boundary(n);
}

void advect(float *p, float *n, float *vel_x, float *vel_y, float dt) {
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
            float s1 = traced_x - x0;
            float s0 = 1.0f - s1;
            float t1 = traced_y - y0;
            float t0 = 1.0f - t1;
            G(n, x, y) = s0 * (t0 * G(p, x0, y0) + t1 * G(p, x0, y1)) +
                         s1 * (t0 * G(p, x1, y0) + t1 * G(p, x1, y1));
        }
    }
    reset_boundary(n);
}

// makes a spiral centered on the center of display
void setup_velocity(float *vel_x, float *vel_y) {
    for (int y = 0; y < DISPLAY_HEIGHT; y++) {
        for (int x = 0; x < DISPLAY_WIDTH; x++) {
            G(vel_x, x, y) =
                (float)(FORCE_SCALE * (-y + (float)DISPLAY_HEIGHT / 2.0f));
            G(vel_y, x, y) =
                (float)(FORCE_SCALE * (x - (float)DISPLAY_WIDTH / 2.0f));
        }
    }
    printf("vel_x center: %f\n",
           G(vel_x, DISPLAY_WIDTH / 2, DISPLAY_HEIGHT / 2));
}

void visualize_velocity(float *vel_x, float *vel_y) {
    for (int y = 0; y < DISPLAY_HEIGHT; y += 30) {
        for (int x = 0; x < DISPLAY_WIDTH; x += 30) {
            DrawCircle(x, y, 2, BLUE);
            DrawLine(x, y, x + G(vel_x, x, y), y + G(vel_y, x, y), BLUE);
        }
    }
}

int main() {
    Arena arena = {0};
    uint8_t *displayed = arena_alloc(&arena, MEM_SIZE * sizeof(uint8_t));

    float *dens1 = arena_alloc(&arena, MEM_SIZE * sizeof(float));
    float *dens2 = arena_alloc(&arena, MEM_SIZE * sizeof(float));

    float *vel_x = arena_alloc(&arena, MEM_SIZE * sizeof(float));
    float *vel_y = arena_alloc(&arena, MEM_SIZE * sizeof(float));
    setup_velocity(vel_x, vel_y);

    insert_rect(dens1, 200, 200, 40, 40);
    float_to_byte(dens1, displayed);

    Image image = {.data = displayed,
                   .format = PIXELFORMAT_UNCOMPRESSED_GRAYSCALE,
                   .height = SIM_HEIGHT,
                   .width = SIM_WIDTH,
                   .mipmaps = 1};

    InitWindow(DISPLAY_WIDTH, DISPLAY_HEIGHT, "hi");
    Texture2D tex = LoadTextureFromImage(image);
    float dt;
    SetTargetFPS(GetMonitorRefreshRate(GetCurrentMonitor()));
    while (!WindowShouldClose()) {
        dt = GetFrameTime();
        // add sources
        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
            G(dens1, GetMouseX(), GetMouseY()) = MAX_FLOAT_VAL;
        }

        // density step
        diffuse(dens1, dens2, dt);
        advect(dens1, dens2, vel_x, vel_y, dt);
        swap_ptr(dens1, dens2);

        // velocity step

        float_to_byte(dens2, displayed);
        UpdateTexture(tex, displayed);

        BeginDrawing();
        DrawTexture(tex, -1, -1, WHITE);
        visualize_velocity(vel_x, vel_y);
        EndDrawing();
        printf("%.02f\n", dt);
    }
    CloseWindow();
    arena_free(&arena);
    return 0;
}