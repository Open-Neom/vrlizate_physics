// Extracted and improved from vrlizate/lib/physics/rigid_body.dart.
// Copyright Open Neom contributors. Licensed under Apache-2.0.
import 'dart:math' as math;

import 'package:vector_math/vector_math.dart';

/// A render adapter supplies positions in one local, meter-scale world.
abstract interface class PhysicsTransform {
  Vector3 get position;
  set position(Vector3 value);
  void rotate(Quaternion delta);
  void onChanged();
}

/// A standalone pose for simulations that do not have a scene graph.
class PhysicsPose implements PhysicsTransform {
  @override
  Vector3 position;
  Quaternion rotation;

  PhysicsPose({Vector3? position, Quaternion? rotation})
    : position = position?.clone() ?? Vector3.zero(),
      rotation = rotation?.clone() ?? Quaternion.identity();

  @override
  void rotate(Quaternion delta) {
    rotation = delta * rotation;
    rotation.normalize();
  }

  @override
  void onChanged() {}
}

/// A world-space axis-aligned collision box. It is not a mesh collider.
class PhysicsBounds {
  final Vector3 min;
  final Vector3 max;

  PhysicsBounds(Vector3 min, Vector3 max)
    : min = min.clone(),
      max = max.clone() {
    _finiteVector(min, 'min');
    _finiteVector(max, 'max');
    for (var axis = 0; axis < 3; axis++) {
      if (min[axis] > max[axis]) {
        throw ArgumentError('Bounds minimum must not exceed maximum.');
      }
    }
  }

  PhysicsBounds.fromCenterExtents(Vector3 center, Vector3 extents)
    : this(center - extents, center + extents);
}

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

  static final Vector3 gravity = Vector3(0, -9.81, 0);

  PhysicsBody({
    PhysicsTransform? transform,
    Vector3? halfExtents,
    this.boundsProvider,
    this.groundAtPivot = false,
    Vector3? velocity,
    Vector3? angularVelocity,
    this.mass = 1,
    this.restitution = 0.3,
    this.friction = 0.5,
    this.linearDamping = 0.01,
    this.angularDamping = 0.05,
    this.isStatic = false,
    this.useGravity = true,
    this.isSleeping = false,
  }) : transform = transform ?? PhysicsPose(),
       halfExtents = halfExtents?.clone() ?? Vector3.all(0.5),
       velocity = velocity ?? Vector3.zero(),
       angularVelocity = angularVelocity ?? Vector3.zero() {
    _validate();
  }

  double get inverseMass {
    _validateMass();
    return isStatic ? 0 : 1 / mass;
  }

  PhysicsBounds? get bounds => boundsProvider != null
      ? boundsProvider!()
      : PhysicsBounds.fromCenterExtents(transform.position, halfExtents);

  void applyForce(Vector3 force) {
    _finiteVector(force, 'force');
    if (isStatic) return;
    _forceAccumulator.add(force);
    if (force.length2 > 0) isSleeping = false;
  }

  void applyImpulse(Vector3 impulse) {
    _finiteVector(impulse, 'impulse');
    if (isStatic) return;
    velocity += impulse * inverseMass;
    if (impulse.length2 > 0) isSleeping = false;
  }

  /// One explicit integration step. Prefer [PhysicsWorld.update] for a loop.
  void update(double dt) {
    _elapsed(dt);
    if (dt == 0) return;
    _integrate(dt, gravity);
    _forceAccumulator.setZero();
  }

  void _integrate(double dt, Vector3 worldGravity) {
    _validate();
    if (isStatic || isSleeping) return;
    final acceleration = _forceAccumulator * inverseMass;
    if (useGravity) acceleration.add(worldGravity);
    velocity += acceleration * dt;
    velocity *= math.pow(1 - linearDamping, dt * 60).toDouble();
    angularVelocity *= math.pow(1 - angularDamping, dt * 60).toDouble();
    transform.position = transform.position + velocity * dt;
    final angularSpeed = angularVelocity.length;
    if (angularSpeed > 0.001) {
      transform.rotate(
        Quaternion.axisAngle(angularVelocity / angularSpeed, angularSpeed * dt),
      );
    }
    transform.onChanged();
    // An almost-stationary falling body must not sleep in midair.
    if (!useGravity && _atRest) isSleeping = true;
  }

  bool get _atRest =>
      velocity.length2 < 0.0001 && angularVelocity.length2 < 0.0001;

  /// Flat local ground only. A null world ground disables this constraint.
  void resolveGroundCollision(double groundY) {
    _finite(groundY, 'groundY');
    _validate();
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
      if (velocity.y.abs() < 0.1) velocity.y = 0;
    }
    transform.onChanged();
    if (_atRest) isSleeping = true;
  }

  /// Minimum-axis AABB separation and linear impulse; no angular contacts.
  static void resolveCollision(PhysicsBody a, PhysicsBody b) {
    if (identical(a, b) || (a.isStatic && b.isStatic)) return;
    a._validate();
    b._validate();
    final boundsA = a.bounds;
    final boundsB = b.bounds;
    if (boundsA == null || boundsB == null) return;

    var overlap = double.infinity;
    var axis = 0;
    var direction = 1.0;
    for (var i = 0; i < 3; i++) {
      final positive = boundsA.max[i] - boundsB.min[i];
      final negative = boundsB.max[i] - boundsA.min[i];
      if (positive <= 0 || negative <= 0) return;
      final penetration = math.min(positive, negative);
      if (penetration < overlap) {
        overlap = penetration;
        axis = i;
        direction = positive <= negative ? 1 : -1;
      }
    }
    final normal = Vector3.zero()..[axis] = direction;
    final inverseA = a.inverseMass;
    final inverseB = b.inverseMass;
    final inverseTotal = inverseA + inverseB;
    final relativeSpeed = (b.velocity - a.velocity).dot(normal);
    if (relativeSpeed < 0) {
      final restitution = (a.restitution + b.restitution) / 2;
      final impulse = -(1 + restitution) * relativeSpeed / inverseTotal;
      a.applyImpulse(normal * -impulse);
      b.applyImpulse(normal * impulse);
    }
    // Correct overlap even when bodies are already moving apart.
    if (!a.isStatic) {
      a.transform.position -= normal * (overlap * inverseA / inverseTotal);
      a.transform.onChanged();
      a.isSleeping = false;
    }
    if (!b.isStatic) {
      b.transform.position += normal * (overlap * inverseB / inverseTotal);
      b.transform.onChanged();
      b.isSleeping = false;
    }
  }

  void _validateMass() {
    if (!mass.isFinite || mass < 0 || (!isStatic && mass == 0)) {
      throw ArgumentError.value(
        mass,
        'mass',
        'Must be positive for dynamic bodies.',
      );
    }
  }

  void _validate() {
    _validateMass();
    for (final entry in {
      'restitution': restitution,
      'friction': friction,
      'linearDamping': linearDamping,
      'angularDamping': angularDamping,
    }.entries) {
      if (!entry.value.isFinite || entry.value < 0 || entry.value > 1) {
        throw ArgumentError.value(entry.value, entry.key, 'Must be in [0, 1].');
      }
    }
    _finiteVector(velocity, 'velocity');
    _finiteVector(angularVelocity, 'angularVelocity');
    _finiteVector(transform.position, 'position');
    _finiteVector(halfExtents, 'halfExtents');
    if (halfExtents.x < 0 || halfExtents.y < 0 || halfExtents.z < 0) {
      throw ArgumentError('Half extents must be nonnegative.');
    }
  }
}

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
    this.groundY = 0,
    Vector3? gravity,
    this.fixedTimeStep = 1 / 60,
    this.maxSubSteps = 8,
    this.maxFrameTime = 0.25,
    this.maxBodies = 256,
  }) : gravity = gravity?.clone() ?? PhysicsBody.gravity.clone() {
    if (!fixedTimeStep.isFinite ||
        fixedTimeStep <= 0 ||
        !maxFrameTime.isFinite ||
        maxFrameTime < fixedTimeStep ||
        maxSubSteps < 1 ||
        maxSubSteps > 120 ||
        maxBodies < 1 ||
        maxBodies > 4096) {
      throw ArgumentError('Invalid fixed-step or active-body budget.');
    }
    _finiteVector(this.gravity, 'gravity');
    if (groundY != null) _finite(groundY!, 'groundY');
  }

  double get interpolationAlpha => _accumulator / fixedTimeStep;

  void addBody(T body) {
    if (bodies.contains(body)) return;
    if (bodies.length >= maxBodies) {
      throw StateError('Physics body budget exceeded.');
    }
    bodies.add(body);
  }

  void removeBody(T body) => bodies.remove(body);

  void resetAccumulator() {
    _accumulator = 0;
    lastStepCount = 0;
  }

  void update(double elapsed) {
    _elapsed(elapsed);
    _finiteVector(gravity, 'gravity');
    if (bodies.length > maxBodies) {
      throw StateError('Physics body budget exceeded.');
    }
    lastStepCount = 0;
    if (elapsed == 0) return;
    final accepted = math.min(elapsed, maxFrameTime);
    droppedTime += elapsed - accepted;
    _accumulator += accepted;
    while (_accumulator + 1e-12 >= fixedTimeStep &&
        lastStepCount < maxSubSteps) {
      for (final body in bodies) {
        body._integrate(fixedTimeStep, gravity);
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
        body._forceAccumulator.setZero();
      }
    }
    if (_accumulator + 1e-12 >= fixedTimeStep) {
      final wholeSteps = ((_accumulator + 1e-12) / fixedTimeStep).floor();
      final dropped = math.min(_accumulator, wholeSteps * fixedTimeStep);
      droppedTime += dropped;
      _accumulator = math.max(0, _accumulator - dropped);
    }
  }
}

void _elapsed(double value) {
  if (!value.isFinite || value < 0) {
    throw ArgumentError.value(
      value,
      'elapsed',
      'Must be finite and nonnegative.',
    );
  }
}

void _finite(double value, String name) {
  if (!value.isFinite) {
    throw ArgumentError.value(value, name, 'Must be finite.');
  }
}

void _finiteVector(Vector3 value, String name) {
  if (!value.x.isFinite || !value.y.isFinite || !value.z.isFinite) {
    throw ArgumentError.value(value, name, 'Must contain finite coordinates.');
  }
}
