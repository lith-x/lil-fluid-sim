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
Steps:
- add density to x0 buffer (init'd clear, based on some user interaction perhaps)
- add forces to u0/v0 buffers (init'd clear, based on some user interaction perhaps)
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
  - iterate over next[i,j] = (prev[i,j] + a * (next[i,j+1] + next[i,j-1], + next[i+1,j] + next[i-1,j])) / c


- jargony notes:
  - laplacian = "curvature", "spread", the delta between the point i,j/x,y and the average of its neighbors
    - more technically, it's the second derivative of the vector i.e. del^2*f = d^2*f / dx^2 + d^2*f / dy^2
    - laplacian is a continuous thing, so we need to make it discrete. enter: 5-point stencil
    - five point stencil
      - laplacian of f(x,y) is approx: [f(x+h,y) + f(x-h,y) + f(x,y+h) + f(x,y-h) - 4*f(x,y)] / h^2
                                       (x[i+1,j] + x[i-1,j] + x[i,j+1] + x[i,j-1] - 4*x[i,j]) / h^2
        - stride conversion: +1 in indexing == +h in units
      - discrete difference between self and neighbors :)
      - https://en.wikipedia.org/wiki/Five-point_stencil#In_two_dimensions
    - read x[i,j]_n as "value of field x at cell [i,j] at time step n"
    - diffusion equation = fick's second law: https://en.wikipedia.org/wiki/Fick%27s_laws_of_diffusion#Fick%27s_second_law
    - d[density of cell]/dt = density_coefficient * five_point_stencil
      - x[i,j]_(n+1) - x[i,j]_n = dt * density_coeff * five_point_stencil
    - long and short of it:
      - d^2f/dx^2 = (f(x+h_x,y) + f(x-h_x,y) - 2*f(x,y)) / h_x^2
      - d^2f/dy^2 = (f(x,y+h_y) + f(x,y-h_y) - 2*f(x,y)) / h_y^2
      - laplacian f(x,y) approx= d^2f/dx^2 + d^2f/dy^2
      - h_x = (hypothetical # of [unit] in width) / pixel_width
      - h_y = (hypothetical # of [unit] in height) / pixel_height

      - fick's second law: df(x,y)/dt = laplacian f(x,y)
        - df(x,y) = dt * ((f(x+h_x,y) + f(x-h_x,y) - 2*f(x,y)) / h_x^2 + (f(x,y+h_y) + f(x,y-h_y) - 2*f(x,y)) / h_y^2)
        - make continuous discrete: df(x,y) = f(x,y)_(n+1) - f(x,y)_n
        - swap that out, add f(x,y)_n to each side. assume everything on right side is at timestep n
        - f(x,y)_(n+1) = f(x,y) + dt * (f(x+h_x,y) + f(x-h_x,y) - 2*f(x,y)) / h_x^2 + dt * (f(x,y+h_y) + f(x,y-h_y) - 2*f(x,y)) / h_y^2
  - Units:
    - Diffusion constant (d): length^2 / time
    - dt: time
    - density: mass / length^(dimensions)
    - laplacian: length^-2
    - viscosity : length^(dim) / time
*/