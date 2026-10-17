import 'dart:math' as math;

import 'package:vector_math/vector_math.dart';

import '../../utils/constants/vr_physics_constants.dart';
import '../../utils/physics_validator.dart';
import 'physics_bounds.dart';

/// A spherical collision volume in local world meters.
final class PhysicsSphereCollider {
  final Vector3 center;
  final double radius;

  PhysicsSphereCollider({
    required Vector3 center,
    this.radius = VrPhysicsConstants.defaultSphereRadius,
  }) : center = center.clone() {
    PhysicsValidator.validateFiniteVector(
      this.center,
      VrPhysicsConstants.paramCenter,
    );
    if (!radius.isFinite || radius <= 0) {
      throw ArgumentError.value(
        radius,
        VrPhysicsConstants.paramRadius,
        VrPhysicsConstants.errPositiveRadius,
      );
    }
  }

  /// Returns the axis-aligned bounding box enclosing this sphere.
  PhysicsBounds get bounds =>
      PhysicsBounds.fromCenterExtents(center, Vector3.all(radius));

  /// Returns `true` if [point] lies inside or on the surface of this sphere.
  bool containsPoint(Vector3 point) {
    PhysicsValidator.validateFiniteVector(
      point,
      VrPhysicsConstants.paramPosition,
    );
    return center.distanceToSquared(point) <= radius * radius;
  }

  /// Returns `true` if this sphere overlaps or touches [other].
  bool intersectsSphere(PhysicsSphereCollider other) {
    final radiusSum = radius + other.radius;
    return center.distanceToSquared(other.center) <= radiusSum * radiusSum;
  }

  /// Returns `true` if this sphere overlaps or touches the AABB [box].
  bool intersectsBounds(PhysicsBounds box) {
    final clampedX = center.x.clamp(box.min.x, box.max.x);
    final clampedY = center.y.clamp(box.min.y, box.max.y);
    final clampedZ = center.z.clamp(box.min.z, box.max.z);
    final dx = center.x - clampedX;
    final dy = center.y - clampedY;
    final dz = center.z - clampedZ;
    return (dx * dx + dy * dy + dz * dz) <= radius * radius;
  }

  /// Returns the distance along [direction] from [origin] to the nearest
  /// sphere intersection within [maxDistance], or `null` if no hit occurs.
  double? intersectRay(
    Vector3 origin,
    Vector3 direction, {
    double maxDistance = VrPhysicsConstants.defaultMaxRaycastDistance,
  }) {
    PhysicsValidator.validateFiniteVector(
      origin,
      VrPhysicsConstants.paramOrigin,
    );
    PhysicsValidator.validateFiniteVector(
      direction,
      VrPhysicsConstants.paramDirection,
    );
    if (direction.length2 <= 0) {
      throw ArgumentError.value(
        direction,
        VrPhysicsConstants.paramDirection,
        VrPhysicsConstants.errNonZeroDirection,
      );
    }
    if (!maxDistance.isFinite || maxDistance <= 0) {
      throw ArgumentError.value(
        maxDistance,
        VrPhysicsConstants.paramMaxDistance,
        VrPhysicsConstants.errPositiveMaxDistance,
      );
    }

    final dir = direction.normalized();
    final oc = origin - center;
    final b = oc.dot(dir);
    final c = oc.dot(oc) - radius * radius;
    if (c > 0 && b > 0) return null;

    final discriminant = b * b - c;
    if (discriminant < 0) return null;

    final t = math.max(0.0, -b - math.sqrt(discriminant));
    return t <= maxDistance ? t : null;
  }
}
