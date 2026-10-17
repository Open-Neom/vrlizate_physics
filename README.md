# vrlizate_physics

Render-independent local rigid-body physics, raycasting, sphere colliders, trigger volumes, and kinematic character navigation for the VRlizate SDK (`v1.0.0`). Uses pure Dart and `vector_math`; it has no Flutter, authentication, network, Earth, or renderer dependency.

```dart
import 'package:vector_math/vector_math.dart';
import 'package:vrlizate_physics/vrlizate_physics.dart';

final pose = PhysicsPose(position: Vector3(0, 4, 0));
final body = PhysicsBody(transform: pose, halfExtents: Vector3.all(0.5));
final simulation = PhysicsWorld<PhysicsBody>()..addBody(body);
simulation.update(1 / 60);

final hit = simulation.raycast(Vector3(0, 10, 0), Vector3(0, -1, 0));
```

## Architecture (`v1.0.0`)

- `lib/utils/constants/vr_physics_constants.dart`: `VrPhysicsConstants` centralizes all numeric defaults, thresholds, and validation messages.
- `lib/domain/models/`:
  - `PhysicsTransform`: Render adapter interface for local positions and rotations.
  - `PhysicsPose`: Standalone headless transform.
  - `PhysicsBounds`: World-space axis-aligned bounding box (AABB) with containment and intersection checks.
  - `PhysicsBody`: Rigid-body state, forces, impulses, damping, and ground/AABB contact resolution.
  - `PhysicsContact`: Structured minimum-axis AABB contact manifold and impulse solver.
  - `PhysicsRaycastHit`: Raycast intersection result (`body`, `point`, `normal`, `distance`).
  - `PhysicsSphereCollider`: Spherical collision primitive with point, sphere, AABB, and ray queries.
  - `PhysicsTriggerVolume`: Non-solid AABB volume with `containsPoint`, `intersectsBounds`, and occupant enter/exit tracking.
  - `PhysicsWalkRegion` & `PhysicsObstacleBox`: Floor and obstacle primitives for character navigation.
- `lib/domain/services/`:
  - `PhysicsWorld`: Bounded fixed-step simulation loop with `raycast` and `raycastAll`.
  - `PhysicsCharacterSpace`: Bounded kinematic walking and wall-sliding in flat XZ space.

## Verification

Run `flutter test` and `flutter analyze` inside this package.
