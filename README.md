# My fork of SPlisHSPlasH

Just trying to understand things here and writing down what I learn.

## Before you build

Visual Studio: Set configuration to release with debug info (RelWithDebInfo) to get reasonable frametimes.

## Differences to other CFD simulators

There are no meshes and no elements.
Material is still represented by computational nodes.
The computational nodes are particles.

Space is "empty" before material enters.
This is the Lagrangian instead of the Eulerian worldview.

There is no problem domain.
The "problem domain" is infinite, in the sense of floating point infinity.

Free surfaces are possible without multiphase flow.
This is a rather extraterrestrial idea.
For physicality one can and should use multiple phases.

There are no steady states.
You can always watch a simulation evolve.
You could call this transient simulation.

There is no control over convergence.
Fluid equations are fulfilled in different ways to different degrees by the available methods.
This is a bit hard to describe.

There are no boundary conditions and no initial conditions.
No inlets, no walls, no contacts, no outlets.
Instead, there are emitters, animation fields and rigid bodies.

The number of computational nodes is dynamic.
However, one may want to control the scale of the computation.
To this end, emitters can have a maximum number of particles,
and a bounding box, where leaving particles are reused.

There is the concept of scenes instead of cases.
For setup, you can sample cubes, spheres or arbitrary meshes with particles.
Rigid bodies can participate and there is collision detection.

There are no turbulence models, aka. turbulence is always on.
There is no laminar-only default option, as in engineering software.
There is nothing based on Reynolds decomposition (yet?).
Since no simplification like the Reynolds average is used, everything can be considered a large eddy simulation.

Instead of turbulence, the concept of vorticity is used.
Vorticity is a special case of turbulence, that can be quantified and controlled (?).

Fluids can represent elastic bodies.
The constitutive material model is Linear Elasticity (Young's modulus, Poisson's ratio).

## TODO

- Export velocity field to regular grid
  - Seed new particles at grid cell centers
  - Use current SPH method to get the velocity from the pressure solve

- Instant scene preview
  - Watch json file
  - Extract names and geometries
  - Render as schematic view
  - Make it easy to avoid overlaps
  - Python could be nice for this
    - Binding exists: /doc/pysplash/creating_scenes.md
    - manim is nice for text and lines, and animations of course *.*
    - blender can import partio
    - Qt may have what else we need, and can be easily integrated: https://build-system.fman.io/