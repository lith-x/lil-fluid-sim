#include <raylib.h>

#define ARENA_IMPLEMENTATION
#include "arena.h"

#define HEIGHT 512
#define WIDTH 512
#define MEM_SPACE (HEIGHT + 2) * (WIDTH + 2)
#define PIXEL_COUNT HEIGHT *WIDTH
#define MAX_FLOAT_VAL 50.0f
#define DIFF_COEFF 1.0f
#define GAUSS_ITERS 5

// G short for "grid", gives you some (x,y) out of the 1d array "arr"
#define G(arr, x, y) arr[((y) + 1) * (WIDTH) + (x) + 1]

void add_vecs_dt(float *dst, float *src, float dt) {
    for (int i = 0; i < PIXEL_COUNT; i++)
        dst[i] += src[i] * dt;
}

// p short for "previous state", n short for "next state"
// h_[x,y] is ratio: real measurement / pixel in [x,y] direction
void diffuse_unstable(float *p, float *n, float h_x, float h_y,
                      float diff_coeff, float dt) {
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            // prevent boundary crossing with cancelling via negative
            float top = y == 0 ? -G(p, x, y - 1) : G(p, x, y - 1);
            float bottom = y == HEIGHT - 1 ? -G(p, x, y + 1) : G(p, x, y + 1);
            float left = x == 0 ? -G(p, x - 1, y) : G(p, x - 1, y);
            float right = x == WIDTH - 1 ? -G(p, x + 1, y) : G(p, x + 1, y);
            float center = G(p, x, y);
            // Fick's Second Law using Five-Point Stencil for Laplacian
            // approximation:
            // https://en.wikipedia.org/wiki/Fick%27s_laws_of_diffusion#Fick%27s_second_law
            // https://en.wikipedia.org/wiki/Five-point_stencil#In_two_dimensions
            /* f(x,y)_(n+1) = f(x,y) + dt * (f(x+h_x,y) + f(x-h_x,y) - 2*f(x,y))
               / h_x^2 + dt * (f(x,y+h_y) + f(x,y-h_y) - 2*f(x,y)) / h_y^2 */
            G(n, x, y) =
                center + diff_coeff * dt *
                             ((left + right - 2 * center) / (h_x * h_x) +
                              (bottom + top - 2 * center) / (h_y * h_y));
        }
    }
}

// p short for "previous state", n short for "next state"
// h_[x,y] is ratio: (length of [x,y] cell)/([width,height]) <- scales
void diffuse(float *p, float *n, float dt) {
    // "cell" width / total width (cell width is 1px)
    float diff_scale = DIFF_COEFF * dt;
    float a_x = diff_scale * (float)(WIDTH * WIDTH);
    float a_y = diff_scale * (float)(HEIGHT * HEIGHT);
    float denom = 1.0f / (1.0f + 2.0f * (a_x + a_y));

    for (int k = 0; k < GAUSS_ITERS; k++) {
        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                G(n, x, y) = denom * (G(p, x, y) +
                                      a_x * (G(n, x + 1, y) + G(n, x - 1, y)) +
                                      a_y * (G(n, x, y + 1) + G(n, x, y - 1)));
            }
        }
    }
    G(n, WIDTH + 1, HEIGHT + 1) = 0;
    G(n, WIDTH + 1, -1) = 0;
    G(n, -1, HEIGHT + 1) = 0;
    G(n, -1, -1) = 0;
}

// void advect(float *u, float *v) {
//     float dt0 = dt*N;

//     // get previous timestamp as an approximation to current trajectory
//     float x = i - dt0 * G(u, i, j);
//     float y = j - dt0 * G(v, i, j);
//     // clamp the values
//     if (x < 0.5) x = 0.5;
//     if (x > N + 0.5) x = N + 0.5;
//     if (y < 0.5) y = 0.5;
//     if (y > N + 0.5) y = N + 0.5;

//     float i0 = (int)x;
//     float i1 = i0 + 1;
//     float j0 = (int)y;
//     float j1 = j0 + 1;

//     float s1 = x - i0;
//     float s0 = 1 - s1;

//     float t1 = y - j0;
//     float t0 = 1 - t1;

//     d[IX(i, j)] = s0 * (t0 * d0[IX(i0, j0)] + t1 * d 0 [IX(i0, j1)]) +
//                   s1 * (t0 * d0[IX(i1, j0)] + t1 * d 0 [IX(i1, j1)]);
// }

// this puts floats (computation) into bytes (displayed as greyscale), probably
// want to go the other way around if user input for density ever comes along
void floats_to_bytes(uint8_t *dst, float *src) {
    // assumed min = 0
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x <= HEIGHT; x++) {
            G(dst, x, y) = (uint8_t)(G(src, x, y) / MAX_FLOAT_VAL) * UINT8_MAX;
        }
    }
}

void insert_rect(float *arr, int x, int y, int width, int height) {
    for (int ly = y; ly < y + height; ly++) {
        for (int lx = x; lx < x + width; lx++) {
            G(arr, lx, ly) = MAX_FLOAT_VAL;
        }
    }
}

void draw_circle(float *arr, int x, int y, int radius) {
    for (int ly = y - radius; ly <= y + radius; ly++) {
        for (int lx = x - radius; lx <= x + radius; lx++) {
            // (x - a)^2 + (y - b)^2 <= radius^2  ==  circle @ (a,b) w/ raidus
            int c_x = (lx - x) * (lx - x);
            int c_y = (ly - y) * (ly - y);
            if (c_x + c_y <= radius * radius)
                G(arr, lx, ly) = MAX_FLOAT_VAL;
        }
    }
}

int main() {
    Arena arena = {0};
    float *dens1 = arena_alloc(&arena, MEM_SPACE * sizeof(float));
    float *dens2 = arena_alloc(&arena, MEM_SPACE * sizeof(float));
    uint8_t *displayed = arena_alloc(&arena, PIXEL_COUNT * sizeof(uint8_t));

    // draw_rect(dens1, (float)WIDTH / 2.0f - 20, (float)HEIGHT / 2.0f - 20, 40,
    //           40);
    insert_rect(dens1, WIDTH / 2 - 50, HEIGHT / 2 - 50, 100, 100);
    floats_to_bytes(displayed, dens1);

    Image image = {.width = WIDTH,
                   .height = HEIGHT,
                   .mipmaps = 1,
                   .format = PIXELFORMAT_UNCOMPRESSED_GRAYSCALE};

    // latest time InitWindow can be called without causing a crash is before
    // LoadTextureFromImage call
    InitWindow(WIDTH, HEIGHT, "hyello");
    Texture2D tex = LoadTextureFromImage(image);

    void *temp;
    // int fps = GetMonitorRefreshRate(GetCurrentMonitor());
    // SetTargetFPS(fps);
    // float frame_time = 1 / (float)fps;
    while (!WindowShouldClose()) {
        diffuse(dens1, dens2, GetFrameTime());
        floats_to_bytes(displayed, dens2);
        UpdateTexture(tex, displayed);
        BeginDrawing();
        DrawTexture(tex, -1, -1, WHITE);
        EndDrawing();
        temp = dens1;
        dens1 = dens2;
        dens2 = temp;
        printf("frame: %.02f\n", GetFrameTime());
    }
    CloseWindow();
    arena_free(&arena);
    return 0;
}

/*
Steps:
- add density to x0 buffer (init'd clear, based on some user interaction
perhaps)
- add forces to u0/v0 buffers (init'd clear, based on some user interaction
perhaps)
- step
  - velocity step
    - add u0[i] * dt to u[i]
    - add v0[i] * dt to v[i]
    - diffusion on u0/u
    - diffusion on v0/v
    - "project"
    - advect on u0/u
    - advect on v0/v
    - "project"
  - density step
    - add x0[i] * dt to x[i]
    - diffusion on x0/x
    - advect on x0/x
- clear all the "0" arrays
- repeat

- diffusion: the fluid dissipating into the medium
  - set magic constant a = dt * diffusion rate * simulation area
  - linear_solve
- advection: the medium "pushing around" our fluid

- linear_solve
  - method has a name: "iterative stencil loop"
  - iterative approximation via Gauss-Seidel method for efficiency
  - laplacian is like the second derivative for vectors, i.e. "acceleration"
  - c = 1 + 4a
  - iterate over next[i,j] = (prev[i,j] + a * (next[i,j+1] + next[i,j-1], +
    next[i+1,j] + next[i-1,j])) / c

(notes ended up being on diffusion, i started with reminding myself what the
laplacian was)
- laplacian = "curvature", "spread", the delta between the point i,j/x,y and the
  average of its neighbors
  - more technically, it's the second derivative of the vector i.e.
    del^2*f = d^2*f / dx^2 + d^2*f / dy^2
  - laplacian is a continuous thing, so we need to make it discrete. enter:
    5-point stencil
  - five point stencil
    - laplacian of f(x,y) is approx: [f(x+h,y) + f(x-h,y) + f(x,y+h) + f(x,y-h)
- 4*f(x,y)] / h^2 (x[i+1,j] + x[i-1,j] + x[i,j+1] + x[i,j-1] - 4*x[i,j]) / h^2
      - stride conversion: +1 in indexing == +h in units
    - discrete difference between self and neighbors :)
    - https://en.wikipedia.org/wiki/Five-point_stencil#In_two_dimensions
  - read x[i,j]_n as "value of field x at cell [i,j] at time step n"
  - diffusion equation = fick's second law:
    https://en.wikipedia.org/wiki/Fick%27s_laws_of_diffusion#Fick%27s_second_law
  - d[density of cell]/dt = density_coefficient * five_point_stencil
    - x[i,j]_(n+1) - x[i,j]_n = dt * density_coeff * five_point_stencil
  - long and short of it:
    - d^2f/dx^2 = (f(x+h_x,y) + f(x-h_x,y) - 2*f(x,y)) / h_x^2
    - d^2f/dy^2 = (f(x,y+h_y) + f(x,y-h_y) - 2*f(x,y)) / h_y^2
    - laplacian f(x,y) approx= d^2f/dx^2 + d^2f/dy^2
    - h_x = (hypothetical # of [unit] in width) / pixel_width
    - h_y = (hypothetical # of [unit] in height) / pixel_height

    - fick's second law: df(x,y)/dt = laplacian f(x,y)
      - df(x,y) = dt * ((f(x+h_x,y) + f(x-h_x,y) - 2*f(x,y)) / h_x^2 +
                  (f(x,y+h_y) + f(x,y-h_y) - 2*f(x,y)) / h_y^2)
      - make continuous discrete: df(x,y) = f(x,y)_(n+1) - f(x,y)_n
      - swap that out, add f(x,y)_n to each side. assume everything on right
        side is at timestep n
      - f(x,y)_(n+1) = f(x,y) + dt * d * (f(x+h_x,y) + f(x-h_x,y) - 2*f(x,y)) /
                       h_x^2 + dt * d * (f(x,y+h_y) + f(x,y-h_y) - 2*f(x,y)) /
                       h_y^2

- stabilizing diffusion
  - instead of using p to produce n(i,j) (unstable), let's try to make an
    equation that could produce p(i,j) given n ("reversing time" so to speak)
  - don't know *why* this is stable, will look into how this conforms to CFL
    condition?
  - use Gauss-Seidel method/approximation.
    - in a system of equations, start with whatever (can be anything, but if you
      got something closer i.e. a previous solution, it'll converge quicker)
    - if only *other* variables in the system are used to determine the current
      one, you can use the most recently calculated approximations for those
      values to get an approximation of the value we want, and they all will
      converge.
    - there's "Jacobi method" as well, but that method specifically uses the
      previous approximation, as opposed to Gauss-Seidel which uses whatever was
      most recently calculated, which results in slower convergence i.e. needs
      more iterations

  - original: n(i,j) = p(i,j) + dt*diff_coeff*(p(i+1,j) + p(i-1,j) - 2*p(i,j)) /
                       h_x^2 + dt*diff_coeff*(p(i,j+1) + p(i,j-1) - 2*p(i,j)) /
                       h_y^2
  - new: p(i,j) = n(i,j) - dt*diff_coeff*(n(i+1,j) + n(i-1,j) - 2*n(i,j)) /
                  h_x^2 - dt*diff_coeff*(n(i,j+1) + n(i,j-1) - 2*n(i,j)) / h_y^2
    - n(i,j) = p(i,j) + dt*diff_coeff*(n(i+1,j) + n(i-1,j) - 2*n(i,j)) /
               h_x^2 + dt*diff_coeff*(n(i,j+1) + n(i,j-1) - 2*n(i,j)) / h_y^2
  - iterating over this will approximate the correct solution very well (within
    floating point error?), and is stable :D

- advection
  - Advection equation:
    https://en.wikipedia.org/wiki/Advection#The_advection_equation
  - f == density scalar field
  - df/dt + velocity_field (dot) gradient(f) = 0
    - df/dt = -velocity_field (dot) gradient(f)
    - df = dt * -velocity_field (dot) gradient(f)


- stability:
  - need to be constantly enforcing 0 divergence over entire field (no "sources"
    or "sinks" to liquid)
  - The "CFL" condition:
    https://en.wikipedia.org/wiki/Courant%E2%80%93Friedrichs%E2%80%93Lewy_condition


- Units:
  - Diffusion constant (d): length^2 / time
  - dt: time
  - density: mass / length^(dimensions)
  - laplacian: length^-2
  - viscosity : length^(dim) / time
*/