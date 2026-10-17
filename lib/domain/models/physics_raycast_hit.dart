import 'package:vector_math/vector_math.dart';

import 'physics_body.dart';

/// Represents a raycast intersection with a [PhysicsBody].
final class PhysicsRaycastHit {
  final PhysicsBody body;
  final Vector3 point;
  final Vector3 normal;
  final double distance;

  PhysicsRaycastHit({
    required this.body,
    required Vector3 point,
    required Vector3 normal,
    required this.distance,
  }) : point = point.clone(),
       normal = normal.clone();
}
