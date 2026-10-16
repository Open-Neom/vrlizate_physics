import 'dart:math' as math;

import 'package:vector_math/vector_math.dart';

/// A walkable rectangle in local meters. The union can form connected rooms.
final class PhysicsWalkRegion {
  PhysicsWalkRegion(this.minX, this.maxX, this.minZ, this.maxZ) {
    if (![minX, maxX, minZ, maxZ].every((v) => v.isFinite) ||
        minX >= maxX ||
        minZ >= maxZ) {
      throw ArgumentError('Walk regions require finite, ordered bounds.');
    }
  }

  final double minX, maxX, minZ, maxZ;

  bool contains(double x, double z) =>
      x >= minX && x <= maxX && z >= minZ && z <= maxZ;
}

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
      throw ArgumentError('Obstacle boxes require finite positive dimensions.');
    }
  }

  final double x, y, z, width, height, depth;
}

/// Bounded kinematic walking in a flat XZ space, extracted from VRlizate Home.
///
/// It moves a square footprint through a union of floors and slides along static
/// boxes. It does not apply gravity, climb terrain, move dynamic bodies or infer
/// physical tracking. Returned positions keep the caller's Y coordinate.
final class PhysicsCharacterSpace {
  PhysicsCharacterSpace({
    required Iterable<PhysicsWalkRegion> regions,
    required Iterable<PhysicsObstacleBox> obstacles,
    this.bodyRadius = .25,
    this.minSolidHeight = .1,
    this.maxSolidHeight = 1.9,
    this.maxMoveMeters = .5,
    this.maxSubstepMeters = .08,
  }) : regions = List.unmodifiable(regions),
       obstacles = List.unmodifiable(
         obstacles.where(
           (box) =>
               box.y + box.height / 2 > minSolidHeight &&
               box.y - box.height / 2 < maxSolidHeight,
         ),
       ) {
    if (!bodyRadius.isFinite ||
        bodyRadius <= 0 ||
        bodyRadius > .5 ||
        !minSolidHeight.isFinite ||
        !maxSolidHeight.isFinite ||
        minSolidHeight >= maxSolidHeight ||
        !maxMoveMeters.isFinite ||
        maxMoveMeters <= 0 ||
        maxMoveMeters > 1 ||
        !maxSubstepMeters.isFinite ||
        maxSubstepMeters < .01 ||
        maxSubstepMeters > maxMoveMeters) {
      throw ArgumentError('Invalid character-space budget or body dimensions.');
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
      if ((x - box.x).abs() < box.width / 2 + bodyRadius &&
          (z - box.z).abs() < box.depth / 2 + bodyRadius) {
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
