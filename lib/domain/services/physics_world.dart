import 'dart:math' as math;

import 'package:vector_math/vector_math.dart';

import '../../utils/constants/vr_physics_constants.dart';
import '../../utils/physics_validator.dart';
import '../models/physics_body.dart';
import '../models/physics_bounds.dart';
import '../models/physics_raycast_hit.dart';

/// Bounded fixed-step simulation for a small active region, not a whole planet.
///
/// Excess elapsed time is discarded rather than blocking rendering after a
/// pause. [droppedTime] exposes that choice; reset the accumulator on lifecycle
/// suspension. Forces apply through all substeps in the next processed update.
class PhysicsWorld<T extends PhysicsBody> {
  final List<T> bodies = [];
  double? groundY;
  final Vector3 gravity;
  final double fixedTimeStep;
  final int maxSubSteps;
  final double maxFrameTime;
  final int maxBodies;
  double _accumulator = 0;
  double droppedTime = 0;
  int lastStepCount = 0;

  PhysicsWorld({
    this.groundY = VrPhysicsConstants.defaultGroundY,
    Vector3? gravity,
    this.fixedTimeStep = VrPhysicsConstants.defaultFixedTimeStep,
    this.maxSubSteps = VrPhysicsConstants.defaultMaxSubSteps,
    this.maxFrameTime = VrPhysicsConstants.defaultMaxFrameTime,
    this.maxBodies = VrPhysicsConstants.defaultMaxBodies,
  }) : gravity = gravity?.clone() ?? PhysicsBody.gravity.clone() {
    if (!fixedTimeStep.isFinite ||
        fixedTimeStep <= 0 ||
        !maxFrameTime.isFinite ||
        maxFrameTime < fixedTimeStep ||
        maxSubSteps < VrPhysicsConstants.minSubSteps ||
        maxSubSteps > VrPhysicsConstants.maxSubStepsLimit ||
        maxBodies < VrPhysicsConstants.minBodies ||
        maxBodies > VrPhysicsConstants.maxBodiesLimit) {
      throw ArgumentError(VrPhysicsConstants.errInvalidFixedStepOrBudget);
    }
    PhysicsValidator.validateFiniteVector(
      this.gravity,
      VrPhysicsConstants.paramGravity,
    );
    if (groundY != null) {
      PhysicsValidator.validateFinite(
        groundY!,
        VrPhysicsConstants.paramGroundY,
      );
    }
  }

  double get interpolationAlpha => _accumulator / fixedTimeStep;

  void addBody(T body) {
    if (bodies.contains(body)) return;
    if (bodies.length >= maxBodies) {
      throw StateError(VrPhysicsConstants.errBodyBudgetExceeded);
    }
    bodies.add(body);
  }

  void removeBody(T body) => bodies.remove(body);

  void resetAccumulator() {
    _accumulator = 0;
    lastStepCount = 0;
  }

  void update(double elapsed) {
    PhysicsValidator.validateElapsed(elapsed);
    PhysicsValidator.validateFiniteVector(
      gravity,
      VrPhysicsConstants.paramGravity,
    );
    if (bodies.length > maxBodies) {
      throw StateError(VrPhysicsConstants.errBodyBudgetExceeded);
    }
    lastStepCount = 0;
    if (elapsed == 0) return;
    final accepted = math.min(elapsed, maxFrameTime);
    droppedTime += elapsed - accepted;
    _accumulator += accepted;
    while (_accumulator + VrPhysicsConstants.accumulatorEpsilon >=
            fixedTimeStep &&
        lastStepCount < maxSubSteps) {
      for (final body in bodies) {
        body.integrateStep(fixedTimeStep, gravity);
        if (groundY != null) body.resolveGroundCollision(groundY!);
      }
      for (var i = 0; i < bodies.length; i++) {
        for (var j = i + 1; j < bodies.length; j++) {
          PhysicsBody.resolveCollision(bodies[i], bodies[j]);
        }
      }
      _accumulator = math.max(0, _accumulator - fixedTimeStep);
      lastStepCount++;
    }
    if (lastStepCount > 0) {
      for (final body in bodies) {
        body.clearForces();
      }
    }
    if (_accumulator + VrPhysicsConstants.accumulatorEpsilon >= fixedTimeStep) {
      final wholeSteps =
          ((_accumulator + VrPhysicsConstants.accumulatorEpsilon) /
                  fixedTimeStep)
              .floor();
      final dropped = math.min(_accumulator, wholeSteps * fixedTimeStep);
      droppedTime += dropped;
      _accumulator = math.max(0, _accumulator - dropped);
    }
  }

  /// Casts a ray from [origin] along [direction] up to [maxDistance] meters,
  /// returning the closest [PhysicsRaycastHit] against active body bounds, or `null`.
  PhysicsRaycastHit? raycast(
    Vector3 origin,
    Vector3 direction, {
    double maxDistance = VrPhysicsConstants.defaultMaxRaycastDistance,
  }) {
    final hits = raycastAll(origin, direction, maxDistance: maxDistance);
    return hits.isEmpty ? null : hits.first;
  }

  /// Casts a ray from [origin] along [direction] up to [maxDistance] meters,
  /// returning all [PhysicsRaycastHit] results sorted by ascending distance.
  List<PhysicsRaycastHit> raycastAll(
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
    final hits = <PhysicsRaycastHit>[];

    for (final body in bodies) {
      final bounds = body.bounds;
      if (bounds == null) continue;
      final hit = _intersectBounds(body, bounds, origin, dir, maxDistance);
      if (hit != null) {
        hits.add(hit);
      }
    }

    hits.sort((a, b) => a.distance.compareTo(b.distance));
    return List.unmodifiable(hits);
  }

  static PhysicsRaycastHit? _intersectBounds(
    PhysicsBody body,
    PhysicsBounds bounds,
    Vector3 origin,
    Vector3 dir,
    double maxDistance,
  ) {
    var tMin = 0.0;
    var tMax = maxDistance;
    var hitAxis = -1;
    var hitSign = 0.0;

    for (var axis = 0; axis < VrPhysicsConstants.axisCount; axis++) {
      final o = origin[axis];
      final d = dir[axis];
      final minVal = bounds.min[axis];
      final maxVal = bounds.max[axis];

      if (d.abs() < VrPhysicsConstants.rayParallelEpsilon) {
        if (o < minVal || o > maxVal) return null;
      } else {
        final invD = 1.0 / d;
        var t1 = (minVal - o) * invD;
        var t2 = (maxVal - o) * invD;
        var sign = -1.0;
        if (t1 > t2) {
          final temp = t1;
          t1 = t2;
          t2 = temp;
          sign = 1.0;
        }
        if (t1 > tMin) {
          tMin = t1;
          hitAxis = axis;
          hitSign = sign;
        }
        if (t2 < tMax) {
          tMax = t2;
        }
        if (tMin > tMax) return null;
      }
    }

    final normal = hitAxis >= 0
        ? (Vector3.zero()..[hitAxis] = hitSign)
        : -dir;
    final point = origin + dir * tMin;
    return PhysicsRaycastHit(
      body: body,
      point: point,
      normal: normal,
      distance: tMin,
    );
  }
}
