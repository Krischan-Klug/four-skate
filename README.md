# Four Skate Prototype

This repository contains a Unity prototype for a grounded skateboarding experience. The focus of the current iteration is the `BoardController`, a physics-driven controller that keeps the board hovering just above the ground, captures rider input, and forwards recognised tricks to the rest of the gameplay stack.

## Quick Start

1. Add the `BoardController` component to your board root object.
2. Assign the required references:
   - `Board Body`: the board's `Rigidbody` (interpolation enabled, gravity on).
   - `Board Visual Root`: optional transform used for lean visuals (defaults to the component transform).
   - `Input`: the scene's `UnityInputAdapter` instance.
   - `Flick Settings`: a `FlickItSettings` asset for trick recognition.
3. Ensure colliders that should register as ground are on the layers included in `Ground Layers`.
4. Enter Play Mode, hold the left stick to steer, and press the south face button to push.

## Input Mapping

- **Left Stick** (`MoveLS`): steering (X) and pumping/braking (Y).
- **Right Stick** (`TrickRS`): fed into the flick recogniser (currently only used for logging).
- **South Button** (`Push`): queues a ground push.
- **Left / Right Triggers** (`GrabLeft`, `GrabRight`): stored on the last recognised trick event.

These inputs are exposed through `UnityInputAdapter`, which simply wraps `InputActionProperty` bindings so you can remap controls in the Unity Input System.

## BoardController Lifecycle

`Update()` (per rendered frame):
- Caches left-stick input (`MoveInput`).
- Ticks the `FlickItRecognizer`, firing `TrickPerformed` when a gesture resolves.
- Detects the south button press and queues a push for the next physics update.

`FixedUpdate()` (per physics step):
1. `UpdateGroundContact` — Spherecasts to locate the closest ground surface and maintains hover state data (surface normal, distance, grounded flag).
2. `ApplyGravity` — Adds extra gravity in air and applies suspension spring-damper forces while grounded.
3. `ApplySteering` — Rotates the rigidbody towards the desired heading and damps lateral slip.
4. `ApplyPropulsion` — Applies push impulses, pumping acceleration, and braking drag.
5. `ApplyFriction` — Applies rolling resistance when in contact with the ground.
6. `UpdateLeanVisual` — Tilts the visual mesh to match steering input.
7. `UpdateKinematicState` — Updates cached velocity, speed, and airtime values for external systems.

### Public Surface

- **Properties**: `MoveInput`, `Velocity`, `GroundNormal`, `Speed`, `AirTime`, `IsGrounded`, `PopReady`, etc. expose real-time state without re-querying the rigidbody.
- **Events**: `TrickPerformed` delivers a `TrickEvent` populated with recogniser ID, grab status, and whether the gesture was a nollie.
- **Methods**: `ClearPushQueue()` should be called by animation/footstep logic once a push has been consumed to prevent stale input.

## Tunable Parameters

Serialized fields are grouped by category to keep the inspector organised:

### Input & Recognition
- `Flick Settings`, `Input` — references for trick recognition and player input.

### References
- `Board Body`, `Board Visual Root` — physics and visual anchors for the board.

### Ground Detection
- `Ground Layers` — mask for valid riding surfaces.
- `Ground Probe Radius` / `Distance` — size and length of the grounding spherecast.

### Suspension
- `Ride Height` — desired board height above the ground (defaults to **1.1**, as tuned during prototyping).
- `Grounded Buffer` — tolerance above the ride height that still counts as grounded (important for push availability).
- `Suspension Strength` — spring force pulling the board towards the ride height.
- `Suspension Damping` — damping applied along the ground normal when grounded.

### Steering & Lean
- `Turning Rate`, `Air Turning Rate` — yaw responsiveness on ground vs. in air.
- `Orientation Lerp Speed` — how quickly the board re-aligns to its projected forward vector.
- `Lean Angle`, `Lean Response` — visual lean amount and smoothing for the mesh.

### Speed & Push
- `Push Impulse`, `Push Cooldown` — magnitude and minimum delay between pushes.
- `Max Push Speed`, `Max Roll Speed` — soft and hard speed caps.
- `Pump Acceleration`, `Braking Drag` — forward acceleration from pumping and braking resistance when pulling back on the stick.

### Friction & Grip
- `Rolling Resistance` — continual slowdown while grounded.
- `Lateral Grip` — strength of sideways velocity damping.

### Gravity & Air
- `Gravity Multiplier` — extra downward acceleration applied in air.
- `Air Drag` — velocity-based damping while airborne.

### Aerial Requirements
- `Min Pop Speed` — minimum planar speed required for `PopReady` to return true.

## Extending the Controller

- **Animations**: Use `PushQueued` to trigger a push animation, then call `ClearPushQueue()` when the foot makes contact.
- **Tricks**: The controller already captures grabs and nollie state. Extend `TrickEvent` as needed to include spin/flip data once the recogniser provides it.
- **Characters**: Swap `boardVisualRoot` to a rigged character or board mesh to keep physics (collider) and presentation separated.
- **Surfaces**: Adjust `Ground Layers` and probe sizing to support coping, rails, or off-angle surfaces.

## Code Style Notes

- Physics work is isolated in `FixedUpdate` to keep deterministic behaviour.
- All frequently accessed state is cached (speed, velocity, time in air) to avoid repeated projection calculations elsewhere.
- The controller exposes read-only properties instead of public fields to preserve encapsulation and make future refactors safer.

## Troubleshooting

| Symptom | Likely Cause | Recommended Adjustment |
| --- | --- | --- |
| Pushes rarely trigger | `Grounded Buffer` too small or ride height too high | Increase buffer or lower ride height slightly |
| Board feels floaty | `Suspension Strength` too low | Increase strength and damping proportionally |
| Board jitters at rest | Probe radius too small or damping too low | Increase `Ground Probe Radius` / `Suspension Damping` |
| Turns feel sluggish | `Turning Rate` or `Orientation Lerp Speed` too low | Increase gradually (consider `Lean Response` as well) |

## Future Work

- Integrate trick logic (pop detection, spin accounting, trick state machine).
- Layer character animation on top of the hover rig.
- Add terrain-aware friction and downhill acceleration.

Keep this README updated as systems evolve so both code and documentation stay aligned.
