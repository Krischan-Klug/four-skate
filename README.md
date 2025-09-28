# Four Skate

Four Skate is a skate-inspired Unity prototype focused on expressive board control, responsive physics, and a modular gameplay stack. This milestone builds a solid ride feel while keeping the code ready for future systems such as tricks, animation, scoring, and replays.

## Vision And Pillars
- **Authentic Flow**: Maintain a constant sense of motion by blending pushes, pumps, and carving without hard resets.
- **Modular Systems**: Keep input, physics, tricks, and presentation isolated so each can evolve independently.
- **Readable Code**: Prefer descriptive state, events, and documentation so new contributors can extend the project quickly.

## Feature Snapshot (Pre-Alpha)
- Hover based skateboard physics controller with tunable suspension and friction.
- Gamepad friendly control scheme using the Unity Input System.
- Trick gesture recognition pipeline (FlickIt) already wired in for future trick logic.
- Debug friendly telemetry exposed as properties such as `Speed`, `GroundNormal`, and `AirTime`.

## Controls
| Action | Input (Input System) | Notes |
| --- | --- | --- |
| Steer / Pump | `MoveLS` (left stick) | X controls carve, Y pumps forward or brakes |
| Trick Flicks | `TrickRS` (right stick) | Passed directly to the FlickIt recogniser |
| Push | `Push` (south face button) | Queued and executed on the next grounded physics step |
| Left Grab | `GrabLeft` (LT) | Stored on the last trick event |
| Right Grab | `GrabRight` (RT) | Stored on the last trick event |

`UnityInputAdapter` owns the InputActionProperty references so bindings can change without touching gameplay code.

## Project Layout
- `Assets/Scripts/Board/` - Board physics, state, and future board gameplay.
- `Assets/Scripts/Input/` - Input wrappers such as `UnityInputAdapter`.
- `Assets/Scripts/Tricks/` - FlickIt settings and recognition logic.
- `Assets/Scenes/` - Prototype scenes and test setups.
- `README.md` - This document. Keep it in sync with feature work.

Check `ProjectSettings/ProjectVersion.txt` for the Unity version before opening the project.

## System Overview
### Input Layer
- `UnityInputAdapter` exposes strongly typed accessors (`MoveLS`, `TrickRS`, `Push`, `GrabLeft`, `GrabRight`).
- Gameplay reads through the adapter interface to keep controller mappings decoupled from logic.

### Movement And Physics
- `BoardController` owns the rigidbody, tracks ground contact, applies forces, and caches state for other systems.
- `boardVisualRoot` lets us tilt or swap presentation meshes without affecting the physics body.

### Trick Recognition (Work In Progress)
- `FlickItRecognizer` analyses right stick motion and emits `TrickEvent`.
- `BoardController` forwards those events; future systems can interpret, animate, and score them.

## BoardController Architecture
### Responsibilities
- Convert rider input into grounded locomotion and aerial control.
- Maintain a configurable hover offset via a spring and damper.
- Expose rich state (`Velocity`, `GroundNormal`, `PopReady`, etc.) for cameras, UI, and gameplay.
- Dispatch trick events detected by the recogniser.

### Dependencies
- `Rigidbody boardBody` - Physics authority (interpolation enabled, gravity on).
- `UnityInputAdapter input` - Source for rider input.
- `FlickItSettings` and `FlickItRecognizer` - Optional trick recognition.

### Update Flow
```
Update()                            FixedUpdate()
  - Cache MoveLS and queue push      - UpdateGroundContact()
  - Tick FlickIt recogniser          - ApplyGravity()
  - Broadcast trick events           - ApplySteering(dt)
                                      - ApplyPropulsion()
                                      - ApplyFriction()
                                      - UpdateLeanVisual(dt)
                                      - UpdateKinematicState()
```
`Update()` never mutates the rigidbody; it only caches input and handles recogniser output. `FixedUpdate()` owns every physics side effect so the simulation stays deterministic.

### Data Flow Between Scripts
1. `UnityInputAdapter` reads device input.
2. `BoardController.Update()` copies left stick and push values and ticks the recogniser with right stick motion.
3. When a trick resolves, `BoardController` caches `TrickEvent` and fires `TrickPerformed`.
4. `FixedUpdate()` consumes cached input, applies forces, and refreshes public properties for interested systems (camera, UI, trick logic).

### Subsystems Inside BoardController
- **Ground Detection**: `UpdateGroundContact` sphere casts below the centre of mass, captures surface normal and distance, sets `hasGroundContact`, `isGrounded`, and resets `airTime` when needed.
- **Suspension**: `ApplyGravity` adds extra gravity in air and, when grounded, applies spring force `(rideHeight - groundDistance) * suspensionStrength` plus damping along the normal (`normalVelocity * suspensionDamping`).
- **Steering And Lean**: `ApplySteering` yaws the rigidbody around the ground normal, aligns forward direction, damps lateral slip, and `UpdateLeanVisual` tilts the presentation mesh.
- **Propulsion**: `ApplyPropulsion` processes push requests (respecting cooldown, speed caps, and grounded tolerance), adds pump acceleration, and applies braking drag when the stick is pulled back.
- **Friction And Drag**: `ApplyFriction` adds rolling resistance on contact; `ApplyGravity` handles air drag when not grounded.
- **State Caching**: `UpdateKinematicState` caches velocity, planar speed, and airtime so external code avoids repeated projections.

### Extensibility Hooks
- **Animation**: Trigger a push animation when `PushQueued` is true, then call `ClearPushQueue()` once the animation consumes the push.
- **Tricks**: Subscribe to `TrickPerformed` and extend `TrickEvent` with spin or flip data as recognition improves.
- **Camera And FX**: Align cameras or particle effects using `GroundNormal` and `Velocity`.
- **Surface Variations**: Adjust `Ground Layers` or suspension settings at runtime for bowls, rails, or special zones.

## Serialized Tuning Reference
### Input And Recognition
- `Flick Settings`: Reference to the FlickItSettings asset that defines gesture recognition behaviour.
- `Input`: UnityInputAdapter instance that exposes Input System actions to gameplay code.

### References
- `Board Body`: Rigidbody that receives all physics forces and motion.
- `Board Visual Root`: Transform for the visual mesh or character that leans independently of the collider.

### Ground Detection
- `Ground Layers`: LayerMask listing every collider that counts as skateable surface.
- `Ground Probe Radius`: Radius of the grounding sphere cast; larger values increase stability on uneven surfaces.
- `Ground Probe Distance`: Maximum distance checked below the board; adjust to match terrain scale.

### Suspension
- `Ride Height`: Target hover offset from the detected ground (prototype tuned to 1.1).
- `Grounded Buffer`: Extra tolerance above ride height that still counts as grounded for pushes and pops.
- `Suspension Strength`: Spring force pulling the board back toward the ride height.
- `Suspension Damping`: Counter force along the ground normal that prevents oscillation.

### Steering And Lean
- `Turning Rate`: Grounded yaw speed when carving.
- `Air Turning Rate`: Torque applied while airborne for correcting board orientation.
- `Orientation Lerp Speed`: Blend rate used to realign the board to its projected forward direction.
- `Lean Angle`: Maximum visual lean applied to the board mesh.
- `Lean Response`: Smoothing factor for how quickly the lean reaches the target angle.
- `Tail Pivot Local`: Local-space offset used when yawing so the board rotates around the rear truck instead of its centre.

### Speed And Push
- `Push Impulse`: Velocity change applied when a push is consumed.
- `Push Cooldown`: Minimum time between pushes to avoid spam.
- `Max Push Speed`: Speed threshold that reduces push strength as you approach it.
- `Max Roll Speed`: Hard planar speed cap enforced after forces are applied.

### Friction And Grip
- `Rolling Resistance`: Baseline slowdown applied whenever the board is grounded.
- `Lateral Grip`: Strength of the force that cancels sideways drift while carving.

### Gravity And Air
- `Gravity Multiplier`: Scales world gravity to make airborne phases heavier or lighter.
- `Air Drag`: Damping applied to velocity while the board is off the ground.

### Aerial Requirements
- `Min Pop Speed`: Minimum planar speed required before the controller reports `PopReady`.

## Development Guidelines
- Keep new features modular; prefer companion components or event subscribers over expanding `BoardController` with unrelated logic.
- Document new parameters and update this README when behaviour changes.
- Tune physics in dedicated test scenes and preserve presets for different surfaces.

## Troubleshooting
| Symptom | Likely Cause | Recommended Adjustment |
| --- | --- | --- |
| Pushes rarely trigger | `Grounded Buffer` too small or ride height too high | Increase `Grounded Buffer` or lower `Ride Height` |
| Board feels floaty | `Suspension Strength` too low | Raise `Suspension Strength` and `Suspension Damping` |
| Board jitters at rest | Probe radius or damping too low | Increase `Ground Probe Radius` and/or `Suspension Damping` |
| Carves feel sluggish | Turning parameters too low | Increase `Turning Rate` or `Orientation Lerp Speed` |
| Pumping has no impact | Pump value too low or board not grounded | Raise `Pump Acceleration` and confirm ground layers |

## Roadmap Ideas
- Pop and landing detection with a full trick state machine.
- Skater animation layer (foot pushes, grabs, board flips).
- Surface specific tuning for rails, bowls, or rough ground.
- Scoring systems, challenges, and session recording.
- Camera suite with follow cams, replays, and cinematic tools.

Keep iterating on both the implementation and this documentation so the project never drifts toward spaghetti code.
