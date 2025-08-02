#include <raylib.h>
#include <stdint.h>
#define ARENA_IMPLEMENTATION
#include "arena.h"

#define DISPLAY_WIDTH 512
#define DISPLAY_HEIGHT 512
#define SIM_WIDTH (DISPLAY_WIDTH + 2)
#define SIM_HEIGHT (DISPLAY_HEIGHT + 2)
#define MEM_SIZE (SIM_WIDTH * SIM_HEIGHT)
#define MAX_FLOAT_VAL 20.0f
#define DIFF_COEFF 19.0f
#define GAUSS_ITERS 5
#define G(arr, x, y) arr[((y) + 1) * SIM_WIDTH + (x) + 1]

void draw_rect(float *arr, int x, int y, int width, int height) {
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

void diffuse(float *p, float *n, float dt) {
    // "cell" width / total width (cell width is 1px)
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
    for (int x = 0; x < DISPLAY_WIDTH; x++) {
        G(n, x, -1) = -G(n, x, -1);
        G(n, x, DISPLAY_HEIGHT + 1) = -G(n, x, DISPLAY_HEIGHT + 1);
    }
    for (int y = 0; y < DISPLAY_HEIGHT; y++) {
        G(n, -1, y) = -G(n, -1, y);
        G(n, DISPLAY_WIDTH + 1, y) = -G(n, DISPLAY_WIDTH + 1, y);
    }
    G(n, DISPLAY_WIDTH + 1, DISPLAY_HEIGHT + 1) = 0;
    G(n, DISPLAY_WIDTH + 1, -1) = 0;
    G(n, -1, DISPLAY_HEIGHT + 1) = 0;
    G(n, -1, -1) = 0;
}

int main() {
    Arena arena = {0};
    float *dens1 = arena_alloc(&arena, MEM_SIZE * sizeof(float));
    float *dens2 = arena_alloc(&arena, MEM_SIZE * sizeof(float));
    uint8_t *displayed = arena_alloc(&arena, MEM_SIZE * sizeof(uint8_t));

    draw_rect(dens1, 0, 0, DISPLAY_WIDTH / 2, DISPLAY_HEIGHT / 2);
    float_to_byte(dens1, displayed);

    Image image = {.data = displayed,
                   .format = PIXELFORMAT_UNCOMPRESSED_GRAYSCALE,
                   .height = SIM_HEIGHT,
                   .width = SIM_WIDTH,
                   .mipmaps = 1};

    InitWindow(DISPLAY_WIDTH, DISPLAY_HEIGHT, "hi");
    Texture2D tex = LoadTextureFromImage(image);
    void *temp;
    while (!WindowShouldClose()) {
        diffuse(dens1, dens2, GetFrameTime());
        float_to_byte(dens2, displayed);
        UpdateTexture(tex, displayed);
        BeginDrawing();
        DrawTexture(tex, -1, -1, WHITE);
        EndDrawing();
        temp = dens1;
        dens1 = dens2;
        dens2 = temp;
    }
    CloseWindow();
    arena_free(&arena);
    return 0;
}