import 'package:vector_math/vector_math.dart';

import 'constants/vr_physics_constants.dart';

abstract final class PhysicsValidator {
  static void validateElapsed(double value) {
    if (!value.isFinite || value < 0) {
      throw ArgumentError.value(
        value,
        VrPhysicsConstants.paramElapsed,
        VrPhysicsConstants.errFiniteNonNegative,
      );
    }
  }

  static void validateFinite(double value, String name) {
    if (!value.isFinite) {
      throw ArgumentError.value(value, name, VrPhysicsConstants.errFinite);
    }
  }

  static void validateFiniteVector(Vector3 value, String name) {
    if (!value.x.isFinite || !value.y.isFinite || !value.z.isFinite) {
      throw ArgumentError.value(
        value,
        name,
        VrPhysicsConstants.errFiniteCoordinates,
      );
    }
  }
}
