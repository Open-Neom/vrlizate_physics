import 'package:vector_math/vector_math.dart';

import '../../utils/constants/vr_physics_constants.dart';
import '../../utils/physics_validator.dart';

/// A world-space axis-aligned collision box. It is not a mesh collider.
class PhysicsBounds {
  final Vector3 min;
  final Vector3 max;

  PhysicsBounds(Vector3 min, Vector3 max)
    : min = min.clone(),
      max = max.clone() {
    PhysicsValidator.validateFiniteVector(min, VrPhysicsConstants.paramMin);
    PhysicsValidator.validateFiniteVector(max, VrPhysicsConstants.paramMax);
    for (var axis = 0; axis < VrPhysicsConstants.axisCount; axis++) {
      if (min[axis] > max[axis]) {
        throw ArgumentError(VrPhysicsConstants.errBoundsMinExceedsMax);
      }
    }
  }

  PhysicsBounds.fromCenterExtents(Vector3 center, Vector3 extents)
    : this(center - extents, center + extents);

  Vector3 get center => (min + max) / VrPhysicsConstants.halfDivisor;

  Vector3 get size => max - min;

  bool containsPoint(Vector3 point) {
    PhysicsValidator.validateFiniteVector(
      point,
      VrPhysicsConstants.paramPosition,
    );
    return point.x >= min.x &&
        point.x <= max.x &&
        point.y >= min.y &&
        point.y <= max.y &&
        point.z >= min.z &&
        point.z <= max.z;
  }

  bool intersects(PhysicsBounds other) {
    return min.x <= other.max.x &&
        max.x >= other.min.x &&
        min.y <= other.max.y &&
        max.y >= other.min.y &&
        min.z <= other.max.z &&
        max.z >= other.min.z;
  }
}
