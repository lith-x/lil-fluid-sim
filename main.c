#include <raylib.h>
#include <stdint.h>
#include <string.h>

#define ARENA_IMPLEMENTATION
#include "arena.h"

#define HEIGHT 512
#define WIDTH 512
#define PIXEL_COUNT WIDTH *HEIGHT
#define PIXEL(arr, x, y) arr[(y) * WIDTH + (x)]
#define DIFF_RATE 1.0f
/*
Rotational field velocities: (u,v) = (x + y, y - x)
c_1 = a_2 * b_3 - a_3 * b_2
c_2 = a_3 * b_1 - a_1 * b_3

*/

typedef struct velocity_t {
    double u;
    double v;
} velocity_t;

void write_block(uint8_t *arr, int x, int y, int width, int height) {
    for (int ly = y; ly < y + height; ly++) {
        for (int lx = x; lx < x + width; lx++) {
            PIXEL(arr, lx, ly) = 255;
        }
    }
}

void diffuse(uint8_t *prev, uint8_t *next, double dt) {
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            uint8_t left = x == 0 ? 0 : PIXEL(prev, x - 1, y);
            uint8_t right = x == WIDTH - 1 ? 0 : PIXEL(prev, x + 1, y);
            uint8_t top = y == 0 ? 0 : PIXEL(prev, x, y - 1);
            uint8_t bottom = y == 0 ? 0 : PIXEL(prev, x, y + 1);
            PIXEL(next, x, y) =
                PIXEL(prev, x, y) +
                DIFF_RATE * dt *
                    (left + right + top + bottom - 4 * PIXEL(prev, x, y));
        }
    }
}

void velocity_swirl_setup(velocity_t *arr, double force_scale) {
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            PIXEL(arr, x, y) =
                (velocity_t){force_scale * (y - (double)HEIGHT / 2.0)
                             /*force_scale * (y - (double)HEIGHT / 2.0f) + x*/,
                             force_scale * (-x + (double)WIDTH / 2.0)
                             /*force_scale * (-x + (double)WIDTH / 2.0f) + y*/};
        }
    }
}

void advect(uint8_t *current, uint8_t *next, velocity_t *forces, double dt) {
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            velocity_t f = PIXEL(forces, x, y);
            int new_x = x - f.u * dt;
            int new_y = y - f.v * dt;
            PIXEL(next, x, y) += PIXEL(current, new_x < 0 ? 0 : new_x >= WIDTH ? 0 : new_x, new_y < 0 ? 0 : new_y >= HEIGHT ? 0 : new_y);
        }
    }
}

// void compress_data(uint8_t *arr, uint8_t *comp) {
//     double comp_rate = (double)sizeof(uint8_t) / (double)sizeof(uint8_t);
//     for (int i = 0; i < PIXEL_COUNT; i++)
//         comp[i] = arr[i] * comp_rate;
// }

double lerp(double start, double end, double t) {
    return start * (1 - t) + end * t;
}

int main() {
    // Data setup
    Arena arena = {0};
    uint8_t *prev_dens = arena_alloc(&arena, PIXEL_COUNT * sizeof(uint8_t));
    memset(prev_dens, 0, PIXEL_COUNT * sizeof(uint8_t));
    uint8_t *next_dens = arena_alloc(&arena, PIXEL_COUNT * sizeof(uint8_t));
    memset(next_dens, 0, PIXEL_COUNT * sizeof(uint8_t));
    velocity_t *forces = arena_alloc(&arena, PIXEL_COUNT * sizeof(velocity_t));
    velocity_swirl_setup(forces, 50.0f);
    InitWindow(WIDTH, HEIGHT, "mah fluid sim");
    PIXEL(prev_dens, WIDTH / 2, 0) = 255;
    // write_block(prev_dens, 10, 10, 250, 250);
    Image image = {.data = next_dens,
                   .format = PIXELFORMAT_UNCOMPRESSED_GRAYSCALE,
                   .width = WIDTH,
                   .height = HEIGHT,
                   .mipmaps = 1};
    Texture2D texture = LoadTextureFromImage(image);

    double dt; // seconds
    double fps = 20;//GetMonitorRefreshRate(GetCurrentMonitor());
    SetTargetFPS(fps);

    void *temp;
    while (!WindowShouldClose()) {
        dt = GetFrameTime();
        diffuse(prev_dens, next_dens, dt);
        advect(prev_dens, next_dens, forces, dt);
        UpdateTexture(texture, next_dens);
        // Draw
        BeginDrawing();
        ClearBackground(BLACK);
        DrawTexture(texture, 0, 0, WHITE);
        EndDrawing();
        temp = prev_dens;
        prev_dens = next_dens;
        next_dens = temp;
    }
    CloseWindow();
    arena_free(&arena);
    return 0;
}

/*
Resources:
https://shahriyarshahrabi.medium.com/gentle-introduction-to-fluid-simulation-for-programmers-and-technical-artists-7c0045c40bac

*/