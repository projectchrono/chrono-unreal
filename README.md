# Chrono-Unreal
[![BSD License](http://www.projectchrono.org/assets/logos/chrono-bsd.svg)](LICENSE)

A minimal Unreal Engine 5.8 project that runs [Project Chrono](https://projectchrono.org) 10.0.0 vehicle dynamics inside Unreal. A
primitive test car drives on a Nanite landscape that deforms under its wheels, and Chrono can use either Unreal's **Chaos** physics
scene or Chrono's own **Bullet** as its collision backend. The same project doubles as a test bench that compares the two backends.

Chrono is compiled from source into two engine plugins; the rest of the repository is a small game module, a level or three and the
scripts that build them. A longer write-up with diagrams is in [`Docs/UnrealChrono_Architecture.pdf`](Docs/UnrealChrono_Architecture.pdf).

## What is in the box

| | |
|---|---|
| `Plugins/ChronoCore` | Chrono core (bundled Bullet, GIMPACT, HACD, Eigen), `FChronoSimulation` (hosts a `ChSystem` on the Chaos physics thread) and the Chaos collision backend |
| `Plugins/ChronoVehicle` | the `chrono_vehicle` module (wheeled vehicles, suspensions, tires, powertrain, drivers) |
| `Source/UnrealChrono` | the game module: `TestCar`, `ACarPhysicsWorld`, `APhysicsVehicle`, terrain deformation, Bullet level export, the backend comparison test |
| `Content/Maps` | `TerrainDeformTest` (drive on deformable sand), `BackendComparison`, `RigidColliders` |
| `Scripts` | Python that generates the levels (`create_test_level.py`, `create_comparison_level.py`, `create_collider_level.py`) |

## Getting started

Requirements: Windows, Unreal Engine 5.8, Visual Studio with the C++ workload, a DX12 / SM6 GPU for the deformation level.

1. Right click `UnrealChrono.uproject` > *Generate Visual Studio project files*, then build the `UnrealChronoEditor` target
   (Development Editor, Win64). The first build compiles Chrono, which takes several minutes.
2. Open the project. `/Game/Maps/TerrainDeformTest` is the startup map; press Play.

| Key | Action |
|---|---|
| W / S | throttle / brake |
| A / D | steer |
| Space | handbrake |
| R | toggle reverse / drive |

The collision backend is chosen on the `CarPhysicsWorld` actor in the level (`CollisionBackend`: Chaos or Bullet).

## How Chrono runs inside Unreal

The whole `chrono::ChSystem` lives inside a **Chaos sim callback** on Unreal's physics thread, the same mechanism Epic's ChaosVehicles
plugin uses. With *Tick Physics Async* on, the physics thread ticks at a fixed 120 Hz independent of the frame rate and Chrono takes
three 1/360 s substeps per tick. The game thread never touches the `ChSystem`, and the physics thread never touches UObjects; only
plain data crosses:

* **in:** driver inputs (idempotent state, pushed every frame) and one-shot commands (build the vehicle, shift gear);
* **out:** an immutable `FVehicleSnapshot` per physics step (chassis and wheel transforms, per-tire telemetry, ground samples),
  which `APhysicsVehicle` applies to its meshes.

`ACarPhysicsWorld` owns the simulation, `FChronoVehicleWorld` (a stepper on the physics thread) builds and advances the vehicle, driver
and terrain every substep. Chrono is Z-up, right handed, metres; Unreal is Z-up, left handed, centimetres, so the Y axis is mirrored and
lengths scaled (`FChronoConversions`).

## The Chaos collision backend

With `EChronoCollisionBackend::Chaos` the Chrono collision system is replaced by one that delegates to Unreal's physics scene.
Chrono keeps its solver (NSC, PSOR), integrators and contact materials; collision detection is done by Chaos:

* every Chrono collision model is mirrored as a **kinematic proxy particle** in the Chaos solver (sphere, box, capsule, convex hull,
  triangle mesh; a cylinder becomes a 24-sided prism);
* Chrono ↔ Chrono pairs use a sort-and-sweep broadphase and Chaos' GJK / EPA and manifold narrowphase;
* Chrono ↔ Unreal pairs are gathered once per Chaos step from the solver's acceleration structure, so Chrono bodies collide directly
  with the **level** (static meshes, landscape heightfields) without exporting anything;
* ray, sweep and overlap queries run against the same world, which is how the tires sense the ground.

Which Unreal shapes the car sees is decided by **collision channels**: the proxies sit in the `Vehicle` channel (configurable on
`ACarPhysicsWorld`) and block every channel except their own, so an Unreal shape takes part if it blocks `Vehicle`. The visual
vehicle meshes have no collision, so there is no self collision or self sensing.

The **Bullet** backend is the alternative: Chrono's bundled Bullet only sees Chrono bodies, so level geometry tagged `ChronoStatic`
(on the actor *and* the component) is exported to static Chrono bodies and streamed in and out around the vehicle.

## Physical materials

Surfaces are described by `UVehiclePhysicalMaterial` (a `UPhysicalMaterial` with a surface type), assigned to the landscape or to a
static mesh component as usual.

* **Chaos:** friction and restitution of the Unreal shape's material become the Chrono contact material (composed with the other
  body's material by `min`). The surface type reaches the physics thread through a pointer-identity table captured on the game thread,
  so no UObject is touched there.
* **Bullet:** the exporter builds the Chrono material of each static body from the same asset.
* **Used by:** the tire models (friction and surface type) and the terrain deformation (only soft surfaces deform).

## Tire models

Selected with the `TireModel` property on the vehicle:

| Model | Description |
|---|---|
| **Discrete** (default) | Tread-element tire (120 elements x 5 strips) sampling the ground at 3 x 15 points, so it envelopes obstacles. Brush-type friction on hard surfaces, Bekker / Wong / Janosi-Hanamoto terramechanics on soft ones. The only model that publishes its ground heightfield and so drives the terrain deformation. |
| **TMeasy** | Chrono's TMeasy force-element tire, parameters guessed from the tire size and load; senses the ground with a ray. |
| **Rigid** | A cylinder collision shape that collides through the collision backend itself. |

## Terrain deformation

A GPU height-delta system turns the landscape into soft ground that holds tire tracks:

* each wheel on a soft surface sends a footprint stamp (position, direction, size, sinkage); a worker sweeps it along the wheel's path;
* compute passes dig the rut, deposit the displaced soil as berms, relax slopes to an angle of repose (volume conserving) and press the
  tread pattern into the rut floor; two toroidal windows follow the vehicle (coarse 4096 x 5 cm for long memory, fine 4096 x 0.75 cm for
  tread detail), about 384 MB of render targets regardless of landscape size;
* the landscape material (`MF_TerrainDeform`) turns the result into Nanite displacement and normals;
* a readback of the fine window feeds **berms** (not ruts, or the car digs itself in) back into the Chaos terrain query.

Only Sand, Dirt, Mud, Snow, Grass and Gravel surfaces deform, and the feedback exists on the Chaos backend only.

## Bullet vs Chaos comparison

`AChronoBackendComparison` runs identical scenarios on both backends and compares the recorded trajectories on the same time grid. Inputs
are scripted against simulation time, so the frame rate cannot influence the result. Open `/Game/Maps/BackendComparison` (vehicle with
each tire model, rolling cylinders, sparse drop, a 200-body pile) or `/Game/Maps/RigidColliders` (shape pairs, stacks, ramp slide,
bounce) and press Play, or run headless:

```
UnrealEditor.exe UnrealChrono.uproject /Game/Maps/BackendComparison -game -nullrhi -nosound -ChronoCompareQuit
```

The exit code is 0 when every required check passed. Reports and CSVs go to `Saved/BackendComparison/<timestamp>/`. Findings so far:
the Discrete and TMeasy tires agree between backends (0.21 m and 1.28 m over an 82 m drive); the Chaos 24-sided cylinder rolls about 28%
less far than Bullet's analytic cylinder (which also affects the Rigid tire); a capsule resting on a box gains energy on Bullet; and the
test found that Bullet's ground friction was 0 for materials without an authored `StaticFriction` (fixed in
`FBulletStaticCollision::MakeMaterial`). Details, charts and the pass criteria are in the PDF.

## Regenerating the levels

Close the editor, delete the `.umap`, then run for example:

```
UnrealEditor-Cmd.exe UnrealChrono.uproject -run=pythonscript -script=Scripts/create_test_level.py -unattended -nosplash
```

`UnrealChrono.AutoDrive <seconds>` (console variable) holds full throttle, logs the car once per second, then takes a screenshot and quits.

## License

BSD, see [LICENSE](LICENSE). Project Chrono is copyright of the Project Chrono Development Team.
