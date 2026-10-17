import 'package:vector_math/vector_math.dart';

/// A render adapter supplies positions in one local, meter-scale world.
abstract interface class PhysicsTransform {
  Vector3 get position;
  set position(Vector3 value);
  void rotate(Quaternion delta);
  void onChanged();
}
