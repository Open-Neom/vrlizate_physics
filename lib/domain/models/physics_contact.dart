import 'dart:math' as math;

import 'package:vector_math/vector_math.dart';

import '../../utils/constants/vr_physics_constants.dart';
import 'physics_body.dart';

/// Represents a detected AABB contact between two [PhysicsBody] instances.
final class PhysicsContact {
  final PhysicsBody bodyA;
  final PhysicsBody bodyB;
  final Vector3 normal;
  final double penetrationDepth;

  PhysicsContact({
    required this.bodyA,
    required this.bodyB,
    required Vector3 normal,
    required this.penetrationDepth,
  }) : normal = normal.clone();

  /// Detects a minimum-axis AABB contact between [a] and [b], or returns `null`.
  static PhysicsContact? detect(PhysicsBody a, PhysicsBody b) {
    if (identical(a, b) || (a.isStatic && b.isStatic)) return null;
    a.validateState();
    b.validateState();
    final boundsA = a.bounds;
    final boundsB = b.bounds;
    if (boundsA == null || boundsB == null) return null;

    var overlap = double.infinity;
    var axis = 0;
    var direction = 1.0;
    for (var i = 0; i < VrPhysicsConstants.axisCount; i++) {
      final positive = boundsA.max[i] - boundsB.min[i];
      final negative = boundsB.max[i] - boundsA.min[i];
      if (positive <= 0 || negative <= 0) return null;
      final penetration = math.min(positive, negative);
      if (penetration < overlap) {
        overlap = penetration;
        axis = i;
        direction = positive <= negative ? 1.0 : -1.0;
      }
    }
    final normal = Vector3.zero()..[axis] = direction;
    return PhysicsContact(
      bodyA: a,
      bodyB: b,
      normal: normal,
      penetrationDepth: overlap,
    );
  }

  /// Resolves linear velocity impulse and positional overlap between [bodyA] and [bodyB].
  void resolve() {
    final inverseA = bodyA.inverseMass;
    final inverseB = bodyB.inverseMass;
    final inverseTotal = inverseA + inverseB;
    if (inverseTotal <= 0) return;

    final relativeSpeed = (bodyB.velocity - bodyA.velocity).dot(normal);
    if (relativeSpeed < 0) {
      final restitution =
          (bodyA.restitution + bodyB.restitution) /
          VrPhysicsConstants.restitutionAverageDivisor;
      final impulse = -(1 + restitution) * relativeSpeed / inverseTotal;
      bodyA.applyImpulse(normal * -impulse);
      bodyB.applyImpulse(normal * impulse);
    }
    if (!bodyA.isStatic) {
      bodyA.transform.position -=
          normal * (penetrationDepth * inverseA / inverseTotal);
      bodyA.transform.onChanged();
      bodyA.isSleeping = false;
    }
    if (!bodyB.isStatic) {
      bodyB.transform.position +=
          normal * (penetrationDepth * inverseB / inverseTotal);
      bodyB.transform.onChanged();
      bodyB.isSleeping = false;
    }
  }
}
