import '../../utils/constants/vr_physics_constants.dart';

/// A walkable rectangle in local meters. The union can form connected rooms.
final class PhysicsWalkRegion {
  PhysicsWalkRegion(this.minX, this.maxX, this.minZ, this.maxZ) {
    if (![minX, maxX, minZ, maxZ].every((v) => v.isFinite) ||
        minX >= maxX ||
        minZ >= maxZ) {
      throw ArgumentError(VrPhysicsConstants.errWalkRegionBounds);
    }
  }

  final double minX, maxX, minZ, maxZ;

  bool contains(double x, double z) =>
      x >= minX && x <= maxX && z >= minZ && z <= maxZ;
}
