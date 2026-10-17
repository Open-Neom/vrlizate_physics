import 'package:vector_math/vector_math.dart';

import '../../utils/constants/vr_physics_constants.dart';
import 'physics_body.dart';
import 'physics_bounds.dart';

/// Non-solid AABB trigger region that tracks body enter/exit transitions.
final class PhysicsTriggerVolume {
  final String id;
  final PhysicsBounds bounds;
  final void Function(PhysicsBody body)? onEnter;
  final void Function(PhysicsBody body)? onExit;
  final Set<PhysicsBody> _occupants = <PhysicsBody>{};

  PhysicsTriggerVolume({
    required this.id,
    required this.bounds,
    this.onEnter,
    this.onExit,
  }) {
    if (id.trim().isEmpty) {
      throw ArgumentError.value(
        id,
        VrPhysicsConstants.paramId,
        VrPhysicsConstants.errEmptyTriggerId,
      );
    }
  }

  /// Current bodies overlapping this trigger volume.
  Set<PhysicsBody> get occupants => Set.unmodifiable(_occupants);

  /// Returns `true` if [point] lies within [bounds].
  bool containsPoint(Vector3 point) => bounds.containsPoint(point);

  /// Returns `true` if [other] overlaps [bounds].
  bool intersectsBounds(PhysicsBounds other) => bounds.intersects(other);

  /// Returns `true` if [body] is currently tracked inside this trigger volume.
  bool isOccupant(PhysicsBody body) => _occupants.contains(body);

  /// Evaluates [bodies] against [bounds] and updates enter/exit state.
  ({List<PhysicsBody> entered, List<PhysicsBody> exited}) evaluateBodies(
    Iterable<PhysicsBody> bodies,
  ) {
    final currentInside = <PhysicsBody>{};
    final entered = <PhysicsBody>[];
    final exited = <PhysicsBody>[];

    for (final body in bodies) {
      final bodyBounds = body.bounds;
      final inside = bodyBounds != null
          ? intersectsBounds(bodyBounds)
          : containsPoint(body.transform.position);
      if (inside) {
        currentInside.add(body);
        if (_occupants.add(body)) {
          entered.add(body);
          onEnter?.call(body);
        }
      }
    }

    final previousOccupants = _occupants.toList();
    for (final body in previousOccupants) {
      if (!currentInside.contains(body)) {
        _occupants.remove(body);
        exited.add(body);
        onExit?.call(body);
      }
    }

    return (
      entered: List.unmodifiable(entered),
      exited: List.unmodifiable(exited),
    );
  }

  /// Clears all tracked occupants without invoking [onExit].
  void reset() => _occupants.clear();
}
