import 'dart:math' as math;

import 'package:vector_math/vector_math.dart';

import '../../utils/constants/vr_physics_constants.dart';
import '../models/physics_obstacle_box.dart';
import '../models/physics_walk_region.dart';

/// Bounded kinematic walking in a flat XZ space, extracted from VRlizate Home.
///
/// It moves a square footprint through a union of floors and slides along static
/// boxes. It does not apply gravity, climb terrain, move dynamic bodies or infer
/// physical tracking. Returned positions keep the caller's Y coordinate.
final class PhysicsCharacterSpace {
  PhysicsCharacterSpace({
    required Iterable<PhysicsWalkRegion> regions,
    required Iterable<PhysicsObstacleBox> obstacles,
    this.bodyRadius = VrPhysicsConstants.defaultBodyRadius,
    this.minSolidHeight = VrPhysicsConstants.defaultMinSolidHeight,
    this.maxSolidHeight = VrPhysicsConstants.defaultMaxSolidHeight,
    this.maxMoveMeters = VrPhysicsConstants.defaultMaxMoveMeters,
    this.maxSubstepMeters = VrPhysicsConstants.defaultMaxSubstepMeters,
  }) : regions = List.unmodifiable(regions),
       obstacles = List.unmodifiable(
         obstacles.where(
           (box) =>
               box.y + box.height / VrPhysicsConstants.halfDivisor >
                   minSolidHeight &&
               box.y - box.height / VrPhysicsConstants.halfDivisor <
                   maxSolidHeight,
         ),
       ) {
    if (!bodyRadius.isFinite ||
        bodyRadius <= 0 ||
        bodyRadius > VrPhysicsConstants.maxBodyRadius ||
        !minSolidHeight.isFinite ||
        !maxSolidHeight.isFinite ||
        minSolidHeight >= maxSolidHeight ||
        !maxMoveMeters.isFinite ||
        maxMoveMeters <= 0 ||
        maxMoveMeters > VrPhysicsConstants.maxMoveMetersLimit ||
        !maxSubstepMeters.isFinite ||
        maxSubstepMeters < VrPhysicsConstants.minSubstepMeters ||
        maxSubstepMeters > maxMoveMeters) {
      throw ArgumentError(VrPhysicsConstants.errCharacterSpaceBudget);
    }
  }

  final List<PhysicsWalkRegion> regions;
  final List<PhysicsObstacleBox> obstacles;
  final double bodyRadius, minSolidHeight, maxSolidHeight;
  final double maxMoveMeters, maxSubstepMeters;

  bool canStand(double x, double z) {
    if (!x.isFinite || !z.isFinite) return false;
    // Corners may occupy different adjacent regions; shrinking each rectangle
    // would create artificial walls at otherwise traversable floor seams.
    for (final sx in const [-1, 1]) {
      for (final sz in const [-1, 1]) {
        if (!regions.any(
          (region) => region.contains(x + sx * bodyRadius, z + sz * bodyRadius),
        )) {
          return false;
        }
      }
    }
    for (final box in obstacles) {
      if ((x - box.x).abs() <
              box.width / VrPhysicsConstants.halfDivisor + bodyRadius &&
          (z - box.z).abs() <
              box.depth / VrPhysicsConstants.halfDivisor + bodyRadius) {
        return false;
      }
    }
    return true;
  }

  Vector3 move(Vector3 from, double dx, double dz) {
    if (!dx.isFinite ||
        !dz.isFinite ||
        !from.y.isFinite ||
        !canStand(from.x, from.z)) {
      return from;
    }
    // Normalize before measuring: even a huge finite input cannot overflow
    // dx*dx into NaN movement or create an unbounded number of substeps.
    final largest = math.max(dx.abs(), dz.abs());
    if (largest > 0) {
      final nx = dx / largest, nz = dz / largest;
      final normalizedLength = math.sqrt(nx * nx + nz * nz);
      if (largest > maxMoveMeters / normalizedLength) {
        dx = nx * maxMoveMeters / normalizedLength;
        dz = nz * maxMoveMeters / normalizedLength;
      }
    }
    final steps = math.max(
      1,
      (math.max(dx.abs(), dz.abs()) / maxSubstepMeters).ceil(),
    );
    final stepX = dx / steps, stepZ = dz / steps;
    var x = from.x, z = from.z;
    for (var i = 0; i < steps; i++) {
      if (canStand(x + stepX, z)) x += stepX;
      if (canStand(x, z + stepZ)) z += stepZ;
    }
    return Vector3(x, from.y, z);
  }
}
