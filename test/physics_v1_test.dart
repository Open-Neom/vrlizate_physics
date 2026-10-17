import 'package:test/test.dart';
import 'package:vector_math/vector_math.dart';
import 'package:vrlizate_physics/vrlizate_physics.dart';

void main() {
  group('PhysicsWorld.raycast and PhysicsRaycastHit', () {
    test('returns closest hit with point, normal, and distance', () {
      final near = PhysicsBody(
        transform: PhysicsPose(position: Vector3(0, 0, -3)),
        halfExtents: Vector3.all(0.5),
      );
      final far = PhysicsBody(
        transform: PhysicsPose(position: Vector3(0, 0, -6)),
        halfExtents: Vector3.all(0.5),
      );
      final world = PhysicsWorld<PhysicsBody>(groundY: null)
        ..addBody(far)
        ..addBody(near);

      final hit = world.raycast(Vector3.zero(), Vector3(0, 0, -1));
      expect(hit, isNotNull);
      expect(hit!.body, same(near));
      expect(hit.distance, closeTo(2.5, 1e-6));
      expect(hit.point.z, closeTo(-2.5, 1e-6));
      expect(hit.normal, Vector3(0, 0, 1));

      final all = world.raycastAll(Vector3.zero(), Vector3(0, 0, -1));
      expect(all, hasLength(2));
      expect(all[0].body, same(near));
      expect(all[1].body, same(far));
    });

    test('respects maxDistance and returns null when no body is hit', () {
      final target = PhysicsBody(
        transform: PhysicsPose(position: Vector3(0, 0, -5)),
      );
      final world = PhysicsWorld<PhysicsBody>(groundY: null)..addBody(target);

      expect(
        world.raycast(Vector3.zero(), Vector3(0, 0, -1), maxDistance: 3.0),
        isNull,
      );
      expect(world.raycast(Vector3.zero(), Vector3(1, 0, 0)), isNull);
    });

    test('validates raycast inputs', () {
      final world = PhysicsWorld<PhysicsBody>();
      expect(
        () => world.raycast(Vector3.zero(), Vector3.zero()),
        throwsArgumentError,
      );
      expect(
        () => world.raycast(Vector3.zero(), Vector3(1, 0, 0), maxDistance: 0),
        throwsArgumentError,
      );
    });
  });

  group('PhysicsSphereCollider', () {
    test('checks point containment, sphere/bounds overlap, and ray intersection', () {
      final sphere = PhysicsSphereCollider(
        center: Vector3(0, 2, 0),
        radius: 1.5,
      );
      expect(sphere.containsPoint(Vector3(0, 3.0, 0)), isTrue);
      expect(sphere.containsPoint(Vector3(0, 4.0, 0)), isFalse);

      final overlapping = PhysicsSphereCollider(
        center: Vector3(2, 2, 0),
        radius: 1.0,
      );
      final disjoint = PhysicsSphereCollider(
        center: Vector3(4, 2, 0),
        radius: 1.0,
      );
      expect(sphere.intersectsSphere(overlapping), isTrue);
      expect(sphere.intersectsSphere(disjoint), isFalse);

      final box = PhysicsBounds.fromCenterExtents(
        Vector3(1.2, 2, 0),
        Vector3.all(0.5),
      );
      expect(sphere.intersectsBounds(box), isTrue);

      final hitDist = sphere.intersectRay(
        Vector3(0, 2, 5),
        Vector3(0, 0, -1),
      );
      expect(hitDist, closeTo(3.5, 1e-6));
      expect(
        sphere.intersectRay(Vector3(0, 2, 5), Vector3(0, 1, 0)),
        isNull,
      );
    });

    test('rejects non-positive or non-finite radius', () {
      expect(
        () => PhysicsSphereCollider(center: Vector3.zero(), radius: 0),
        throwsArgumentError,
      );
      expect(
        () => PhysicsSphereCollider(
          center: Vector3.zero(),
          radius: double.nan,
        ),
        throwsArgumentError,
      );
    });
  });

  group('PhysicsTriggerVolume', () {
    test('tracks entered and exited bodies across evaluations', () {
      final enteredIds = <PhysicsBody>[];
      final exitedIds = <PhysicsBody>[];
      final trigger = PhysicsTriggerVolume(
        id: 'portal-zone',
        bounds: PhysicsBounds.fromCenterExtents(
          Vector3.zero(),
          Vector3.all(2),
        ),
        onEnter: enteredIds.add,
        onExit: exitedIds.add,
      );

      expect(trigger.containsPoint(Vector3(1, 1, 1)), isTrue);
      expect(trigger.containsPoint(Vector3(3, 0, 0)), isFalse);

      final body = PhysicsBody(
        transform: PhysicsPose(position: Vector3(0, 0, 0)),
      );
      final step1 = trigger.evaluateBodies([body]);
      expect(step1.entered, [body]);
      expect(step1.exited, isEmpty);
      expect(trigger.isOccupant(body), isTrue);
      expect(enteredIds, [body]);

      // Still inside -> no duplicate enter event
      final step2 = trigger.evaluateBodies([body]);
      expect(step2.entered, isEmpty);
      expect(step2.exited, isEmpty);

      // Move outside
      body.transform.position = Vector3(10, 0, 0);
      final step3 = trigger.evaluateBodies([body]);
      expect(step3.entered, isEmpty);
      expect(step3.exited, [body]);
      expect(trigger.isOccupant(body), isFalse);
      expect(exitedIds, [body]);
    });

    test('rejects empty trigger id', () {
      expect(
        () => PhysicsTriggerVolume(
          id: '   ',
          bounds: PhysicsBounds.fromCenterExtents(
            Vector3.zero(),
            Vector3.all(1),
          ),
        ),
        throwsArgumentError,
      );
    });
  });
}
