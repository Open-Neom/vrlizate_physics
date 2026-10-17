import 'dart:math' as math;

import 'package:vector_math/vector_math.dart';

import '../../utils/constants/vr_physics_constants.dart';
import '../../utils/physics_validator.dart';
import 'physics_bounds.dart';
import 'physics_contact.dart';
import 'physics_pose.dart';
import 'physics_transform.dart';

/// Lightweight rigid-body state. Velocities are m/s and rad/s; mass is kg.
///
/// Damping is the fraction lost per 1/60 s, retaining the original VRlizate
/// tuning while scaling correctly with elapsed time. [friction] retains the
/// legacy meaning: tangential velocity retained on a ground contact (0–1).
class PhysicsBody {
  final PhysicsTransform transform;
  final Vector3 halfExtents;
  final PhysicsBounds? Function()? boundsProvider;

  /// Compatibility option for old experiences whose ground aligns the pivot.
  /// New bodies use the bottom of their collision box by default.
  final bool groundAtPivot;

  Vector3 velocity;
  Vector3 angularVelocity;
  double mass;
  double restitution;
  double friction;
  double linearDamping;
  double angularDamping;
  bool isStatic;
  bool useGravity;
  bool isSleeping;
  final Vector3 _forceAccumulator = Vector3.zero();

  static final Vector3 gravity = Vector3(
    0,
    VrPhysicsConstants.defaultGravityY,
    0,
  );

  PhysicsBody({
    PhysicsTransform? transform,
    Vector3? halfExtents,
    this.boundsProvider,
    this.groundAtPivot = false,
    Vector3? velocity,
    Vector3? angularVelocity,
    this.mass = VrPhysicsConstants.defaultMass,
    this.restitution = VrPhysicsConstants.defaultRestitution,
    this.friction = VrPhysicsConstants.defaultFriction,
    this.linearDamping = VrPhysicsConstants.defaultLinearDamping,
    this.angularDamping = VrPhysicsConstants.defaultAngularDamping,
    this.isStatic = false,
    this.useGravity = true,
    this.isSleeping = false,
  }) : transform = transform ?? PhysicsPose(),
       halfExtents =
           halfExtents?.clone() ??
           Vector3.all(VrPhysicsConstants.defaultHalfExtent),
       velocity = velocity ?? Vector3.zero(),
       angularVelocity = angularVelocity ?? Vector3.zero() {
    validateState();
  }

  double get inverseMass {
    _validateMass();
    return isStatic ? 0 : 1 / mass;
  }

  PhysicsBounds? get bounds => boundsProvider != null
      ? boundsProvider!()
      : PhysicsBounds.fromCenterExtents(transform.position, halfExtents);

  void applyForce(Vector3 force) {
    PhysicsValidator.validateFiniteVector(force, VrPhysicsConstants.paramForce);
    if (isStatic) return;
    _forceAccumulator.add(force);
    if (force.length2 > 0) isSleeping = false;
  }

  void applyImpulse(Vector3 impulse) {
    PhysicsValidator.validateFiniteVector(
      impulse,
      VrPhysicsConstants.paramImpulse,
    );
    if (isStatic) return;
    velocity += impulse * inverseMass;
    if (impulse.length2 > 0) isSleeping = false;
  }

  void clearForces() => _forceAccumulator.setZero();

  /// One explicit integration step. Prefer [PhysicsWorld.update] for a loop.
  void update(double dt) {
    PhysicsValidator.validateElapsed(dt);
    if (dt == 0) return;
    integrateStep(dt, gravity);
    clearForces();
  }

  /// Integrates velocity and pose over [dt] with [worldGravity] without clearing forces.
  void integrateStep(double dt, Vector3 worldGravity) {
    validateState();
    if (isStatic || isSleeping) return;
    final acceleration = _forceAccumulator * inverseMass;
    if (useGravity) acceleration.add(worldGravity);
    velocity += acceleration * dt;
    velocity *= math
        .pow(1 - linearDamping, dt * VrPhysicsConstants.dampingReferenceHz)
        .toDouble();
    angularVelocity *= math
        .pow(1 - angularDamping, dt * VrPhysicsConstants.dampingReferenceHz)
        .toDouble();
    transform.position = transform.position + velocity * dt;
    final angularSpeed = angularVelocity.length;
    if (angularSpeed > VrPhysicsConstants.minAngularSpeedForRotation) {
      transform.rotate(
        Quaternion.axisAngle(angularVelocity / angularSpeed, angularSpeed * dt),
      );
    }
    transform.onChanged();
    // An almost-stationary falling body must not sleep in midair.
    if (!useGravity && _atRest) isSleeping = true;
  }

  bool get _atRest =>
      velocity.length2 < VrPhysicsConstants.restVelocitySquaredThreshold &&
      angularVelocity.length2 < VrPhysicsConstants.restVelocitySquaredThreshold;

  /// Flat local ground only. A null world ground disables this constraint.
  void resolveGroundCollision(double groundY) {
    PhysicsValidator.validateFinite(groundY, VrPhysicsConstants.paramGroundY);
    validateState();
    if (isStatic) return;
    final bottom = groundAtPivot
        ? transform.position.y
        : bounds?.min.y ?? transform.position.y;
    if (bottom >= groundY) return;
    final position = transform.position.clone();
    position.y += groundY - bottom;
    transform.position = position;
    if (velocity.y < 0) {
      velocity = Vector3(
        velocity.x * friction,
        -velocity.y * restitution,
        velocity.z * friction,
      );
      if (velocity.y.abs() < VrPhysicsConstants.groundBounceStopVelocity) {
        velocity.y = 0;
      }
    }
    transform.onChanged();
    if (_atRest) isSleeping = true;
  }

  /// Minimum-axis AABB separation and linear impulse; no angular contacts.
  static void resolveCollision(PhysicsBody a, PhysicsBody b) {
    final contact = PhysicsContact.detect(a, b);
    contact?.resolve();
  }

  void _validateMass() {
    if (!mass.isFinite || mass < 0 || (!isStatic && mass == 0)) {
      throw ArgumentError.value(
        mass,
        VrPhysicsConstants.paramMass,
        VrPhysicsConstants.errMassPositiveDynamic,
      );
    }
  }

  void validateState() {
    _validateMass();
    for (final entry in {
      VrPhysicsConstants.paramRestitution: restitution,
      VrPhysicsConstants.paramFriction: friction,
      VrPhysicsConstants.paramLinearDamping: linearDamping,
      VrPhysicsConstants.paramAngularDamping: angularDamping,
    }.entries) {
      if (!entry.value.isFinite || entry.value < 0 || entry.value > 1) {
        throw ArgumentError.value(
          entry.value,
          entry.key,
          VrPhysicsConstants.errUnitInterval,
        );
      }
    }
    PhysicsValidator.validateFiniteVector(
      velocity,
      VrPhysicsConstants.paramVelocity,
    );
    PhysicsValidator.validateFiniteVector(
      angularVelocity,
      VrPhysicsConstants.paramAngularVelocity,
    );
    PhysicsValidator.validateFiniteVector(
      transform.position,
      VrPhysicsConstants.paramPosition,
    );
    PhysicsValidator.validateFiniteVector(
      halfExtents,
      VrPhysicsConstants.paramHalfExtents,
    );
    if (halfExtents.x < 0 || halfExtents.y < 0 || halfExtents.z < 0) {
      throw ArgumentError(VrPhysicsConstants.errHalfExtentsNonNegative);
    }
  }
}
