# vrlizate_physics

Render-independent local rigid-body physics for the VRlizate SDK. Extracted from
the existing `vrlizate` simulator, which now delegates through a `Node` adapter.
Uses Dart and `vector_math`; it has no Flutter, authentication, network, Earth,
or renderer dependency.

```dart
import 'package:vector_math/vector_math.dart';
import 'package:vrlizate_physics/vrlizate_physics.dart';

final pose = PhysicsPose(position: Vector3(0, 4, 0));
final body = PhysicsBody(transform: pose, halfExtents: Vector3.all(0.5));
final simulation = PhysicsWorld<PhysicsBody>()..addBody(body);
simulation.update(1 / 60);
```

## Placement, physics and rendering

The host keeps a bounded map `Map<String, PhysicsBody>` keyed by
`VrWorldEntity.id`. Its `VrWorldAddress` must identify the same world/instance as
the local simulation. Create the body's pose from the entity's local-meter
position, update the simulation, then replace the entity transform with the
body's new pose and pass it to the renderer. This avoids another identity system
or coupling physics to the scene graph. `vrlizate_world` owns placements and
queries, physics evolves active body state, and the renderer consumes poses.
The host decides which nearby entities are active and who may update them.

Use a separate simulation for each local region/instance. Geographic coordinates
must first be projected into a nearby meter-scale frame by `vrlizate_earth`.
Latitude/longitude and planet-sized ECEF coordinates are not physics positions.
Changing a local origin requires coherently rebasing entities, bodies, camera,
and audio; this module does not automatically stream or rebase a planet.

## Current behavior

- Semi-implicit Euler integration, gravity in m/s², mass in kg, impulses in N·s,
  and externally supplied angular velocity in rad/s. No torque/inertia solver.
- Fixed 1/60 s steps, at most eight per update; at most 250 ms of elapsed time is
  accepted. Remaining whole steps are dropped and counted by `droppedTime`.
  `lastStepCount` and `interpolationAlpha` expose timing. No background catch-up
  after suspension; call `resetAccumulator()` at lifecycle boundaries.
- Forces persist through every substep in the next update that actually steps;
  then they clear. Reapply continuous forces once per processed simulation
  update; use `applyImpulse` for instantaneous velocity changes. Applying a force
  every render callback faster than physics ticks accumulates those calls.
- Damping is the fraction lost per 1/60 s; it scales with elapsed time. At 60 Hz
  existing damping tuning is unchanged. `friction` preserves the older meaning:
  fraction of tangential velocity retained on flat-ground contact.
- Ground is optional (`groundY: null`). New standalone bodies rest with their
  AABB bottom on the plane. Legacy `RigidBody(node: ...)` retains pivot-on-ground
  behavior so existing demonstrations do not change their floor offsets.
- AABB minimum-axis contact, restitution, mass-weighted overlap separation, and
  static bodies. Ground contacts only bounce when moving downward. Impulses and
  forces wake sleeping bodies; falling bodies do not sleep in midair.
- Default active-body limit 256, configurable up to 4096. Pair checks are O(n²).
  Those are validation bounds, **not a claim that thousands of dynamic bodies
  run in real time on a phone**. Duplicate `addBody` calls are idempotent.

`PhysicsTransform` allows a renderer to supply a pose without owning simulation
logic; `PhysicsPose` works headlessly. `boundsProvider` supplies current world
AABBs, or `halfExtents` creates a translation-only box. Rotation does not rotate
that default box. Existing `vrlizate` nodes provide transformed world bounds.
Body positions/linear velocities are world-local; the legacy Node adapter retains
its local angular-rotation convention. Avoid scaled/rotated physics parents for
new experiences; a scene root with identity transform is simplest.

`PhysicsCharacterSpace` also contains Home's existing walking collision policy:
unions of walkable X/Z regions, clearance from obstacle boxes, bounded movement
substeps and sliding along walls. Home supplies its room and architecture data
through this API. This keeps plaza navigation inside the same module boundary
without inserting a rigid-body simulation into the camera controller.

## Limits and next stages

This is a discrete small-scene solver: fast bodies can tunnel; rotated boxes are
approximated by AABBs; stacks have no iterative manifold solver. It does not yet
implement swept rigid-body collision, triangle-mesh terrain, dynamic character
controllers (jumping, slopes and steps), joints,
vehicles, fluids, buoyancy, wind/cloth, biological ecosystems or authoritative
multiplayer. Those capabilities belong behind this module's boundary and need
separate implementations and device budgets. The extracted walking controller
does not silently add a second physics loop to the plaza.

Run `dart pub get`, `dart test` and `dart analyze` inside this package.
