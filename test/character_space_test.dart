import 'package:test/test.dart';
import 'package:vector_math/vector_math.dart';
import 'package:vrlizate_physics/src/character_space.dart';

void main() {
  PhysicsCharacterSpace space({List<PhysicsObstacleBox> boxes = const []}) =>
      PhysicsCharacterSpace(
        regions: [PhysicsWalkRegion(-5, 5, -5, 5)],
        obstacles: boxes,
      );

  test('adjacent floors are a union, outer edges retain body clearance', () {
    final world = PhysicsCharacterSpace(
      regions: [
        PhysicsWalkRegion(-2, 0, -2, 2),
        PhysicsWalkRegion(0, 2, -2, 2),
      ],
      obstacles: [],
    );
    expect(world.canStand(0, 0), isTrue);
    expect(world.canStand(1.9, 0), isFalse);
    expect(world.move(Vector3(-.2, 1.6, 0), .4, 0).x, closeTo(.2, 1e-6));
  });

  test('thin wall blocks crossing while allowing a slide', () {
    final world = space(
      boxes: [
        PhysicsObstacleBox(x: .6, y: 1, z: 0, width: .02, height: 2, depth: 5),
      ],
    );
    var point = Vector3(0, 1.6, 0);
    for (var i = 0; i < 12; i++) {
      point = world.move(point, .2, .1);
    }
    expect(point.x, lessThan(.6 - .25 - .01));
    expect(point.z, closeTo(1.2, 1e-6));
    expect(point.y, closeTo(1.6, 1e-6));
  });

  test('floor and high ceiling do not block the standing volume', () {
    final world = space(
      boxes: [
        PhysicsObstacleBox(x: 0, y: -.1, z: 0, width: 9, height: .2, depth: 9),
        PhysicsObstacleBox(x: 0, y: 3, z: 0, width: 9, height: .2, depth: 9),
      ],
    );
    expect(world.canStand(0, 0), isTrue);
    expect(world.obstacles, isEmpty);
  });

  test(
    'bounded movement preserves direction even for extreme finite inputs',
    () {
      final world = space();
      final moved = world.move(Vector3.zero(), 1e308, 1e308);
      expect(moved.length, closeTo(.5, 1e-6));
      expect(moved.x, closeTo(moved.z, 1e-10));
      expect(world.canStand(moved.x, moved.z), isTrue);
      final original = Vector3.zero();
      expect(identical(world.move(original, double.nan, 1), original), isTrue);
      expect(world.canStand(double.infinity, 0), isFalse);
    },
  );

  test('invalid dimensions and work budgets fail before simulation', () {
    expect(() => PhysicsWalkRegion(2, 1, -1, 1), throwsArgumentError);
    expect(
      () => PhysicsObstacleBox(x: 0, y: 0, z: 0, width: 0, height: 1, depth: 1),
      throwsArgumentError,
    );
    expect(
      () => PhysicsCharacterSpace(
        regions: [],
        obstacles: [],
        maxSubstepMeters: .00001,
      ),
      throwsArgumentError,
    );
    expect(() => space().regions.clear(), throwsUnsupportedError);
  });
}
