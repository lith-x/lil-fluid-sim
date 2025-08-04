#include <raylib.h>

#define N 512

int main() {
    InitWindow(N, N, "hi");
    while (!WindowShouldClose()) {
        BeginDrawing();
        EndDrawing();
    }
    CloseWindow();
    return 0;
}