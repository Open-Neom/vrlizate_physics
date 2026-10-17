# Changelog

## 1.0.0

- Modularize package into Open Neom architecture (`lib/domain/models/`, `lib/domain/services/`, `lib/utils/constants/`) with one class per file and backward-compatible `lib/src/` barrel exports.
- Centralize all numeric defaults, parameter names, and error messages in `VrPhysicsConstants`.
- Extract `PhysicsContact` for structured AABB contact detection and impulse/penetration resolution.
- Add `PhysicsRaycastHit` and `PhysicsWorld.raycast` / `PhysicsWorld.raycastAll` for slab-based ray-AABB queries.
- Add `PhysicsSphereCollider` with point containment, sphere/AABB overlap checks, and ray intersection.
- Add `PhysicsTriggerVolume` with `containsPoint`, `intersectsBounds`, and stateful occupant enter/exit tracking.

## 0.1.0

- Extract existing VRlizate rigid-body simulation into a pure-Dart SDK module.
- Add bounded fixed-step updates, per-world gravity and headless transforms.
- Correct force wakeup, elapsed-time damping, AABB overlap direction and
  mass-weighted separation while retaining the public scene adapter.
- Define explicit active-region budgets and deferred full-physics capabilities.
