import 'package:vector_math/vector_math.dart';

import 'physics_transform.dart';

/// A standalone pose for simulations that do not have a scene graph.
class PhysicsPose implements PhysicsTransform {
  @override
  Vector3 position;
  Quaternion rotation;

  PhysicsPose({Vector3? position, Quaternion? rotation})
    : position = position?.clone() ?? Vector3.zero(),
      rotation = rotation?.clone() ?? Quaternion.identity();

  @override
  void rotate(Quaternion delta) {
    rotation = delta * rotation;
    rotation.normalize();
  }

  @override
  void onChanged() {}
}
