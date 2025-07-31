#include <raylib.h>

#define ARENA_IMPLEMENTATION
#include "arena.h"

#define HEIGHT 512
#define WIDTH 512
#define PIXEL_COUNT HEIGHT * WIDTH

void add_vecs_dt(float *dst, float *src, float dt) {
    for (int i = 0; i < PIXEL_COUNT; i++)
        dst[i] += src[i] * dt;
}

int main() {
    InitWindow(WIDTH, HEIGHT, "hyello");
    while (!WindowShouldClose()) {
        BeginDrawing();
        EndDrawing();
    }
    CloseWindow();
    return 0;
}

/*


*/