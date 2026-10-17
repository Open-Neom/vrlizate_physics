import '../../utils/constants/vr_physics_constants.dart';

/// Static axis-aligned solid shared by a host's render and character controller.
final class PhysicsObstacleBox {
  PhysicsObstacleBox({
    required this.x,
    required this.y,
    required this.z,
    required this.width,
    required this.height,
    required this.depth,
  }) {
    if (![x, y, z, width, height, depth].every((v) => v.isFinite) ||
        width <= 0 ||
        height <= 0 ||
        depth <= 0) {
      throw ArgumentError(VrPhysicsConstants.errObstacleDimensions);
    }
  }

  final double x, y, z, width, height, depth;
}
