import 'package:test/test.dart';
import 'package:vector_math/vector_math.dart';
import 'package:vrlizate_physics/vrlizate_physics.dart';

PhysicsBody bodyAt(
  double x, {
  double y = 2,
  double mass = 1,
  double restitution = 0,
  bool isStatic = false,
  bool useGravity = false,
}) => PhysicsBody(
  transform: PhysicsPose(position: Vector3(x, y, 0)),
  mass: mass,
  restitution: restitution,
  isStatic: isStatic,
  useGravity: useGravity,
  linearDamping: 0,
  angularDamping: 0,
);

void main() {
  group('body state and forces', () {
    test('gravity accelerates bodies equally regardless of mass', () {
      final light = bodyAt(0, mass: 1, useGravity: true);
      final heavy = bodyAt(2, mass: 10, useGravity: true);
      light.update(0.1);
      heavy.update(0.1);
      expect(light.velocity.y, closeTo(-0.981, 1e-6));
      expect(heavy.velocity.y, closeTo(light.velocity.y, 1e-6));
    });

    test('static zero-mass bodies ignore forces and impulses', () {
      final body = bodyAt(0, mass: 0, isStatic: true, useGravity: true);
      body.applyForce(Vector3.all(10));
      body.applyImpulse(Vector3.all(10));
      body.update(1);
      expect(body.inverseMass, 0);
      expect(body.transform.position.y, 2);
      expect(body.velocity.length, 0);
    });

    test('force wakes a sleeping body and clears after integration', () {
      final body = bodyAt(0, mass: 2)..isSleeping = true;
      body.applyForce(Vector3(4, 0, 0));
      expect(body.isSleeping, isFalse);
      body.update(0.5);
      expect(body.velocity.x, closeTo(1, 1e-6));
      body.update(0.5);
      expect(body.velocity.x, closeTo(1, 1e-6));
    });

    test('impulse response depends on mass and wakes the body', () {
      final body = bodyAt(0, mass: 2)..isSleeping = true;
      body.applyImpulse(Vector3(6, 0, 0));
      expect(body.velocity.x, 3);
      expect(body.isSleeping, isFalse);
    });

    test('damping has the same velocity decay at 30 and 120 Hz', () {
      final slow = bodyAt(0)..linearDamping = 0.01;
      final fast = bodyAt(0)..linearDamping = 0.01;
      slow.velocity.x = fast.velocity.x = 10;
      for (var i = 0; i < 30; i++) {
        slow.update(1 / 30);
      }
      for (var i = 0; i < 120; i++) {
        fast.update(1 / 120);
      }
      expect(slow.velocity.x, closeTo(fast.velocity.x, 1e-4));
    });

    test('a tiny falling step cannot put an airborne body to sleep', () {
      final body = bodyAt(0, useGravity: true);
      body.update(0.0001);
      expect(body.isSleeping, isFalse);
    });

    test('angular velocity integrates a normalized quaternion', () {
      final pose = PhysicsPose();
      final body = PhysicsBody(
        transform: pose,
        angularVelocity: Vector3(0, 1, 0),
        angularDamping: 0,
        useGravity: false,
      );
      body.update(0.5);
      expect(pose.rotation.y, closeTo(0.2474039, 1e-6));
      expect(pose.rotation.w, closeTo(0.9689124, 1e-6));
      expect(pose.rotation.length, closeTo(1, 1e-6));
    });

    test('invalid mutable state cannot contaminate the simulation', () {
      expect(() => bodyAt(0, mass: 0), throwsArgumentError);
      expect(() => bodyAt(0, restitution: 2), throwsArgumentError);
      final body = bodyAt(0);
      expect(() => body.update(double.nan), throwsArgumentError);
      expect(() => body.update(-1), throwsArgumentError);
      expect(
        () => body.applyForce(Vector3(double.infinity, 0, 0)),
        throwsArgumentError,
      );
      body.mass = 0;
      expect(() => body.update(1 / 60), throwsArgumentError);
    });
  });

  group('contacts', () {
    test(
      'new box bodies rest on their bottom, legacy pivots stay supported',
      () {
        final box = bodyAt(0, y: -1)..velocity.y = -1;
        box.resolveGroundCollision(0);
        expect(box.transform.position.y, 0.5);
        expect(box.velocity.y, 0);
        final pivot = PhysicsBody(
          transform: PhysicsPose(position: Vector3(0, -1, 0)),
          groundAtPivot: true,
          restitution: 0,
        );
        pivot.resolveGroundCollision(0);
        expect(pivot.transform.position.y, 0);
      },
    );

    test('ground correction does not reverse a body moving upward', () {
      final body = bodyAt(0, y: -1)..velocity.y = 2;
      body.resolveGroundCollision(0);
      expect(body.velocity.y, 2);
      expect(body.isSleeping, isFalse);
    });

    test('head-on elastic collision exchanges equal-mass velocities', () {
      final a = bodyAt(0, restitution: 1)..velocity.x = 2;
      final b = bodyAt(0.8, restitution: 1)..velocity.x = -1;
      PhysicsBody.resolveCollision(a, b);
      expect(a.velocity.x, closeTo(-1, 1e-6));
      expect(b.velocity.x, closeTo(2, 1e-6));
      expect(b.transform.position.x - a.transform.position.x, closeTo(1, 1e-6));
    });

    test('static collision corrects full overlap along the nearest axis', () {
      final wall = bodyAt(0, isStatic: true);
      final mover = bodyAt(0.8, y: 2.1)..velocity.x = -2;
      PhysicsBody.resolveCollision(wall, mover);
      expect(wall.transform.position.x, 0);
      expect(mover.transform.position.x, closeTo(1, 1e-6));
      expect(mover.transform.position.y, closeTo(2.1, 1e-6));
      expect(mover.velocity.x, 0);
    });

    test('separating bodies still get mass-weighted overlap correction', () {
      final a = bodyAt(0, mass: 1)..velocity.x = -1;
      final b = bodyAt(0.8, mass: 3)..velocity.x = 1;
      PhysicsBody.resolveCollision(a, b);
      expect(a.transform.position.x, closeTo(-0.15, 1e-6));
      expect(b.transform.position.x, closeTo(0.85, 1e-6));
      expect(a.velocity.x, -1);
      expect(b.velocity.x, 1);
    });

    test('coincident centers separate deterministically without NaN', () {
      final a = bodyAt(0);
      final b = bodyAt(0);
      PhysicsBody.resolveCollision(a, b);
      expect(a.transform.position.x, -0.5);
      expect(b.transform.position.x, 0.5);
      expect(a.velocity.length, 0);
      expect(b.velocity.length, 0);
    });

    test('contained boxes separate entirely using the nearest exit', () {
      final outer = PhysicsBody(halfExtents: Vector3.all(2), isStatic: true);
      final inner = bodyAt(0, y: 0);
      PhysicsBody.resolveCollision(outer, inner);
      expect(inner.transform.position.x, 2.5);
    });

    test('a nullable external collider can disable body contacts', () {
      final ghost = PhysicsBody(boundsProvider: () => null);
      final other = PhysicsBody();
      PhysicsBody.resolveCollision(ghost, other);
      expect(other.transform.position.length, 0);
    });
  });

  group('fixed-step world', () {
    test('different render cadences produce the same simulated position', () {
      final a = bodyAt(0, y: 100, useGravity: true);
      final b = bodyAt(0, y: 100, useGravity: true);
      final slow = PhysicsWorld<PhysicsBody>(groundY: null)..addBody(a);
      final fast = PhysicsWorld<PhysicsBody>(groundY: null)..addBody(b);
      for (var i = 0; i < 30; i++) {
        slow.update(1 / 30);
      }
      for (var i = 0; i < 120; i++) {
        fast.update(1 / 120);
      }
      expect(a.transform.position.y, closeTo(b.transform.position.y, 1e-6));
      expect(a.velocity.y, closeTo(b.velocity.y, 1e-6));
    });

    test('one frame force applies throughout all its fixed substeps', () {
      final body = bodyAt(0);
      final world = PhysicsWorld<PhysicsBody>(groundY: null)..addBody(body);
      body.applyForce(Vector3(60, 0, 0));
      world.update(1 / 30);
      expect(world.lastStepCount, 2);
      expect(body.velocity.x, closeTo(2, 1e-6));
      world.update(1 / 60);
      expect(body.velocity.x, closeTo(2, 1e-6));
    });

    test('a frame without a physics step retains pending forces', () {
      final body = bodyAt(0);
      final world = PhysicsWorld<PhysicsBody>(groundY: null)..addBody(body);
      body.applyForce(Vector3(60, 0, 0));
      world.update(1 / 120);
      expect(world.lastStepCount, 0);
      expect(world.interpolationAlpha, closeTo(0.5, 1e-9));
      world.update(1 / 120);
      expect(body.velocity.x, closeTo(1, 1e-6));
    });

    test('long resume is bounded and does not carry a spiral of debt', () {
      final body = bodyAt(0)..velocity.x = 1;
      final world = PhysicsWorld<PhysicsBody>(groundY: null)..addBody(body);
      world.update(10);
      expect(world.lastStepCount, 8);
      expect(body.transform.position.x, closeTo(8 / 60, 1e-6));
      expect(world.droppedTime, closeTo(10 - 8 / 60, 1e-9));
      expect(world.interpolationAlpha, lessThan(1));
      world.update(1 / 60);
      expect(world.lastStepCount, 1);
    });

    test('per-world gravity supports different local environments', () {
      final body = bodyAt(0, useGravity: true);
      final world = PhysicsWorld<PhysicsBody>(
        groundY: null,
        gravity: Vector3(0, -1, 0),
      )..addBody(body);
      world.update(1 / 60);
      expect(body.velocity.y, closeTo(-1 / 60, 1e-6));
    });

    test('world enforces capacity and add is idempotent', () {
      final body = bodyAt(0);
      final world = PhysicsWorld<PhysicsBody>(maxBodies: 1)..addBody(body);
      world.addBody(body);
      expect(world.bodies, hasLength(1));
      expect(() => world.addBody(bodyAt(2)), throwsStateError);
      world.bodies.add(bodyAt(4));
      expect(() => world.update(1 / 60), throwsStateError);
    });

    test('invalid time and budgets are rejected and accumulator resets', () {
      expect(() => PhysicsWorld(fixedTimeStep: 0), throwsArgumentError);
      expect(() => PhysicsWorld(maxSubSteps: 0), throwsArgumentError);
      expect(() => PhysicsWorld(maxFrameTime: double.nan), throwsArgumentError);
      final world = PhysicsWorld<PhysicsBody>();
      expect(() => world.update(double.infinity), throwsArgumentError);
      world.update(1 / 120);
      world.resetAccumulator();
      expect(world.interpolationAlpha, 0);
    });
  });
}
