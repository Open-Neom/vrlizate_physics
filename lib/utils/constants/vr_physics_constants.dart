abstract final class VrPhysicsConstants {
  static const int axisCount = 3;

  static const double defaultGravityY = -9.81;
  static const double defaultHalfExtent = 0.5;
  static const double defaultMass = 1.0;
  static const double defaultRestitution = 0.3;
  static const double defaultFriction = 0.5;
  static const double defaultLinearDamping = 0.01;
  static const double defaultAngularDamping = 0.05;
  static const double dampingReferenceHz = 60.0;
  static const double minAngularSpeedForRotation = 0.001;
  static const double restVelocitySquaredThreshold = 0.0001;
  static const double groundBounceStopVelocity = 0.1;
  static const double restitutionAverageDivisor = 2.0;

  static const double defaultGroundY = 0.0;
  static const double defaultFixedTimeStep = 1.0 / 60.0;
  static const int defaultMaxSubSteps = 8;
  static const int minSubSteps = 1;
  static const int maxSubStepsLimit = 120;
  static const double defaultMaxFrameTime = 0.25;
  static const int defaultMaxBodies = 256;
  static const int minBodies = 1;
  static const int maxBodiesLimit = 4096;
  static const double accumulatorEpsilon = 1e-12;

  static const double defaultBodyRadius = 0.25;
  static const double maxBodyRadius = 0.5;
  static const double defaultMinSolidHeight = 0.1;
  static const double defaultMaxSolidHeight = 1.9;
  static const double defaultMaxMoveMeters = 0.5;
  static const double maxMoveMetersLimit = 1.0;
  static const double defaultMaxSubstepMeters = 0.08;
  static const double minSubstepMeters = 0.01;
  static const double halfDivisor = 2.0;

  static const double defaultMaxRaycastDistance = 1000.0;
  static const double rayParallelEpsilon = 1e-9;
  static const double defaultSphereRadius = 0.5;

  static const String paramMin = 'min';
  static const String paramMax = 'max';
  static const String paramForce = 'force';
  static const String paramImpulse = 'impulse';
  static const String paramGroundY = 'groundY';
  static const String paramMass = 'mass';
  static const String paramRestitution = 'restitution';
  static const String paramFriction = 'friction';
  static const String paramLinearDamping = 'linearDamping';
  static const String paramAngularDamping = 'angularDamping';
  static const String paramVelocity = 'velocity';
  static const String paramAngularVelocity = 'angularVelocity';
  static const String paramPosition = 'position';
  static const String paramHalfExtents = 'halfExtents';
  static const String paramGravity = 'gravity';
  static const String paramElapsed = 'elapsed';
  static const String paramOrigin = 'origin';
  static const String paramDirection = 'direction';
  static const String paramMaxDistance = 'maxDistance';
  static const String paramCenter = 'center';
  static const String paramRadius = 'radius';
  static const String paramId = 'id';

  static const String errBoundsMinExceedsMax =
      'Bounds minimum must not exceed maximum.';
  static const String errMassPositiveDynamic =
      'Must be positive for dynamic bodies.';
  static const String errUnitInterval = 'Must be in [0, 1].';
  static const String errHalfExtentsNonNegative =
      'Half extents must be nonnegative.';
  static const String errInvalidFixedStepOrBudget =
      'Invalid fixed-step or active-body budget.';
  static const String errBodyBudgetExceeded = 'Physics body budget exceeded.';
  static const String errFiniteNonNegative = 'Must be finite and nonnegative.';
  static const String errFinite = 'Must be finite.';
  static const String errFiniteCoordinates = 'Must contain finite coordinates.';
  static const String errWalkRegionBounds =
      'Walk regions require finite, ordered bounds.';
  static const String errObstacleDimensions =
      'Obstacle boxes require finite positive dimensions.';
  static const String errCharacterSpaceBudget =
      'Invalid character-space budget or body dimensions.';
  static const String errNonZeroDirection =
      'Ray direction must be finite and non-zero.';
  static const String errPositiveMaxDistance =
      'Maximum raycast distance must be finite and positive.';
  static const String errPositiveRadius =
      'Sphere collider radius must be finite and positive.';
  static const String errEmptyTriggerId =
      'Trigger volume id must not be empty.';
}
