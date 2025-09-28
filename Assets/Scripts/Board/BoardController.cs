using UnityEngine;

public sealed class BoardController : MonoBehaviour {
    [Header("Input & Recognition")]
    [SerializeField] FlickItSettings flickSettings;
    [SerializeField] UnityInputAdapter input;

    [Header("References")]
    [SerializeField] Rigidbody boardBody;
    [SerializeField] Transform boardVisualRoot;

    [Header("Ground Detection")]
    [SerializeField] LayerMask groundLayers = ~0;
    [SerializeField, Min(0f)] float groundProbeRadius = 0.25f;
    [SerializeField, Min(0f)] float groundProbeDistance = 0.6f;

    [Header("Suspension")]
    [SerializeField, Min(0f)] float rideHeight = 1.1f;
    [SerializeField, Min(0f)] float groundedBuffer = 0.08f;
    [SerializeField, Min(0f)] float suspensionStrength = 12f;
    [SerializeField, Min(0f)] float suspensionDamping = 30f;

    [Header("Steering & Lean")]
    [SerializeField] float turningRate = 140f;
    [SerializeField] float airTurningRate = 60f;
    [SerializeField] float orientationLerpSpeed = 10f;
    [SerializeField] float leanAngle = 9f;
    [SerializeField] float leanResponse = 12f;
    [SerializeField] Vector3 tailPivotLocal = new Vector3(0f, 0f, -0.45f);

    [Header("Speed & Push")]
    [SerializeField] float pushImpulse = 4.5f;
    [SerializeField] float pushCooldown = 0.5f;
    [SerializeField] float maxPushSpeed = 9f;
    [SerializeField] float maxRollSpeed = 18f;
    [SerializeField] float brakingStrength = 5f;

    [Header("Friction & Grip")]
    [SerializeField] float rollingResistance = 0.2f;
    [SerializeField] float lateralGrip = 35f;

    [Header("Gravity & Air")] 
    [SerializeField] float gravityMultiplier = 1.15f;
    [SerializeField] float airDrag = 0.2f;

    FlickItRecognizer recognizer;
    TrickEvent lastTrick;
    float timeSinceLastTrick;

    Vector2 moveInput;
    bool pushQueued;
    bool pushRequest;
    float lastPushTime;

    Quaternion visualBaseRotation;
    RaycastHit groundHit;
    Vector3 groundNormal = Vector3.up;
    bool hasGroundContact;
    bool isGrounded;
    float groundDistance = float.PositiveInfinity;
    float airTime;
    Vector3 velocity;
    float speed;
    bool trickPoseActive;
    Quaternion trickRotationOffset = Quaternion.identity;
    Vector3 trickPositionOffset = Vector3.zero;
    bool allowTrickPositionOffset;
    Vector3 visualBasePosition;

    public Vector2 MoveInput => moveInput;
    public Vector2 TrickInput => input ? input.TrickRS : Vector2.zero;
    public bool PushPressed => input && input.PushPressed;
    public float GrabLeft => input ? input.GrabLeft : 0f;
    public float GrabRight => input ? input.GrabRight : 0f;

    public bool IsGrounded => isGrounded;
    public bool InAir => !isGrounded;
    public float AirTime => airTime;
    public Transform BoardVisualRoot => boardVisualRoot;
    public Rigidbody BoardBody => boardBody;
    public Quaternion VisualBaseRotation => visualBaseRotation;
    public bool TrickPoseActive => trickPoseActive;
    public float PlanarSpeed => speed;
    public bool PushQueued => pushQueued;
    public bool PushReady => isGrounded && (Time.time - lastPushTime) >= pushCooldown && speed < maxRollSpeed;
    public float PushCooldownRemaining => Mathf.Max(0f, pushCooldown - (Time.time - lastPushTime));
    public bool HasGroundContact => hasGroundContact;
    public float GroundDistance => groundDistance;
    public TrickId CurrentQueuedTrick => lastTrick.id;
    public Vector3 GroundNormal => groundNormal;
    public Vector3 Velocity => velocity;
    public float ActualSpeed => velocity.magnitude;

    public TrickEvent LastTrick => lastTrick;
    public float TimeSinceLastTrick => timeSinceLastTrick;

    public event System.Action<TrickEvent> TrickPerformed;

    void Awake() {
        if (!boardBody)
            boardBody = GetComponent<Rigidbody>();

        if (!boardBody)
            Debug.LogError("BoardController requires a Rigidbody assigned.", this);

        if (!boardVisualRoot)
            boardVisualRoot = transform;

        allowTrickPositionOffset = boardVisualRoot != null && boardVisualRoot != transform;

        visualBaseRotation = boardVisualRoot.localRotation;
        visualBasePosition = boardVisualRoot.localPosition;
        trickRotationOffset = Quaternion.identity;
        trickPositionOffset = Vector3.zero;
        trickPoseActive = false;

        if (flickSettings)
            recognizer = new FlickItRecognizer(flickSettings);
        else
            Debug.LogWarning("BoardController: FlickItSettings missing.", this);

        if (boardBody) {
            boardBody.interpolation = RigidbodyInterpolation.Interpolate;
            boardBody.useGravity = true;
            boardBody.constraints |= RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        }
    }

    void OnValidate() {
        if (!boardVisualRoot)
            boardVisualRoot = transform;

        allowTrickPositionOffset = boardVisualRoot != null && boardVisualRoot != transform;

        if (boardVisualRoot) {
            visualBaseRotation = boardVisualRoot.localRotation;
            visualBasePosition = boardVisualRoot.localPosition;
        }
    }

    void Update() {

        float dt = Time.deltaTime;
        timeSinceLastTrick += dt;
        moveInput = input ? Vector2.ClampMagnitude(input.MoveLS, 1f) : Vector2.zero;

        if (input && recognizer != null && recognizer.Tick(TrickInput, dt, out var trick, out bool nollie) && trick != TrickId.None) {
            lastTrick = new TrickEvent {
                id = trick,
                nollie = nollie,
                grabLeft = GrabLeft > 0.1f,
                grabRight = GrabRight > 0.1f,
                spinDegrees = 0
            };
            timeSinceLastTrick = 0f;
            TrickPerformed?.Invoke(lastTrick);
            Debug.Log($"[Board] Flick recognized {trick} nollie={nollie}", this);
        }

        if (PushPressed && hasGroundContact) {
            pushQueued = true;
            pushRequest = true;
        } else if (!hasGroundContact) {
            pushQueued = false;
            pushRequest = false;
        }
    }

    void FixedUpdate() {
        if (!boardBody)
            return;

        float dt = Time.fixedDeltaTime;

        UpdateGroundContact(dt);
        ApplyGravity();
        ApplySteering(dt);
        ApplyPropulsion();
        ApplyFriction();
        UpdateLeanVisual(dt);
        UpdateKinematicState();
    }







    void UpdateGroundContact(float dt) {
        Vector3 origin = boardBody.worldCenterOfMass + Vector3.up * groundProbeRadius;
        hasGroundContact = Physics.SphereCast(origin, groundProbeRadius, Vector3.down, out groundHit, groundProbeDistance, groundLayers, QueryTriggerInteraction.Ignore);

        if (hasGroundContact) {
            groundNormal = groundHit.normal.normalized;
            Vector3 centerToPoint = boardBody.worldCenterOfMass - groundHit.point;
            groundDistance = Mathf.Max(0f, Vector3.Dot(centerToPoint, groundNormal));
            isGrounded = groundDistance <= rideHeight + groundedBuffer;
        } else {
            groundNormal = Vector3.up;
            groundDistance = float.PositiveInfinity;
            isGrounded = false;
        }

        if (isGrounded)
            airTime = 0f;
        else
            airTime += dt;
    }

    void ApplyGravity() {
        if (!boardBody)
            return;

        Vector3 extraGravity = Physics.gravity * (gravityMultiplier - 1f);
        if (extraGravity.sqrMagnitude > 0f)
            boardBody.AddForce(extraGravity, ForceMode.Acceleration);

        if (!hasGroundContact) {
            boardBody.AddForce(-boardBody.linearVelocity * airDrag, ForceMode.Acceleration);
            return;
        }

        float heightError = rideHeight - groundDistance;
        Vector3 springForce = groundNormal * (heightError * suspensionStrength);
        float normalVelocity = Vector3.Dot(boardBody.linearVelocity, groundNormal);
        Vector3 dampingForce = -groundNormal * normalVelocity * suspensionDamping;

        boardBody.AddForce(springForce + dampingForce, ForceMode.Acceleration);
    }

    void ApplySteering(float dt) {
        if (!boardBody)
            return;

        if (trickPoseActive)
            return;

        float steer = Mathf.Clamp(moveInput.x, -1f, 1f);

        if (hasGroundContact) {
            Quaternion currentRotation = boardBody.rotation;
            Quaternion desiredRotation = currentRotation;

            Vector3 planarVelocity = Vector3.ProjectOnPlane(boardBody.linearVelocity, groundNormal);

            if (Mathf.Abs(steer) > 0.001f) {
                Quaternion yawDelta = Quaternion.AngleAxis(steer * turningRate * dt, groundNormal);
                desiredRotation = yawDelta * desiredRotation;

                if (tailPivotLocal.sqrMagnitude > 1e-6f) {
                    Vector3 pivotWorld = boardBody.transform.TransformPoint(tailPivotLocal);
                    Vector3 offset = boardBody.position - pivotWorld;
                    Vector3 rotatedOffset = yawDelta * offset;
                    boardBody.MovePosition(pivotWorld + rotatedOffset);
                }
            }

            if (planarVelocity.sqrMagnitude > 0.01f) {
                Vector3 forward = Vector3.ProjectOnPlane(desiredRotation * Vector3.forward, groundNormal);
                if (forward.sqrMagnitude > 1e-4f) {
                    forward.Normalize();
                    Quaternion align = Quaternion.LookRotation(forward, groundNormal);
                    float alignWeight = DampedLerp(orientationLerpSpeed, dt);
                    desiredRotation = Quaternion.Slerp(desiredRotation, align, alignWeight);
                }
            }

            Quaternion finalRotation = Quaternion.Slerp(currentRotation, desiredRotation, Mathf.Clamp01(dt * orientationLerpSpeed));
            boardBody.MoveRotation(finalRotation);

            if (planarVelocity.sqrMagnitude > 0f) {
                Vector3 forwardProjected = Vector3.ProjectOnPlane(boardBody.transform.forward, groundNormal);
                if (forwardProjected.sqrMagnitude > 1e-4f) {
                    forwardProjected.Normalize();
                    float forwardSpeed = Vector3.Dot(planarVelocity, forwardProjected);
                    Vector3 forwardVelocity = forwardProjected * forwardSpeed;
                    Vector3 lateralVelocity = planarVelocity - forwardVelocity;
                    boardBody.AddForce(-lateralVelocity * lateralGrip, ForceMode.Acceleration);
                }
            }
        } else {
            if (Mathf.Abs(steer) > 0.01f)
                boardBody.AddTorque(Vector3.up * steer * airTurningRate, ForceMode.Acceleration);
        }
    }

    void ApplyPropulsion() {
        Vector3 forward = Vector3.ProjectOnPlane(boardBody.transform.forward, groundNormal);
        if (forward.sqrMagnitude < 1e-4f)
            forward = Vector3.ProjectOnPlane(transform.forward, Vector3.up);

        if (forward.sqrMagnitude < 1e-4f)
            forward = Vector3.forward;

        forward.Normalize();

        Vector3 planarVelocity = Vector3.ProjectOnPlane(boardBody.linearVelocity, groundNormal);
        float planarSpeed = planarVelocity.magnitude;

        bool groundedForPush = hasGroundContact && groundDistance <= rideHeight + groundedBuffer * 1.5f;

        if (pushRequest && groundedForPush) {
            bool cooledDown = Time.time - lastPushTime >= pushCooldown;
            if (cooledDown && planarSpeed < maxRollSpeed) {
                float pushScale = Mathf.Clamp01(1f - planarSpeed / maxPushSpeed);
                if (pushScale > 0.01f) {
                    boardBody.AddForce(forward * pushImpulse * pushScale, ForceMode.VelocityChange);
                    lastPushTime = Time.time;
                }
            }
            pushRequest = false;
            pushQueued = false;
        } else if (!hasGroundContact) {
            pushRequest = false;
            pushQueued = false;
        }

        float pump = Mathf.Clamp(moveInput.y, -1f, 1f);
        if (hasGroundContact && pump < -0.1f && planarVelocity.sqrMagnitude > 0.001f) {
            Vector3 brakingForce = -planarVelocity.normalized * Mathf.Abs(pump) * brakingStrength;
            boardBody.AddForce(brakingForce, ForceMode.Acceleration);
        }
        if (planarSpeed > maxRollSpeed && planarVelocity.sqrMagnitude > 0.001f) {
            Vector3 cappedPlanar = planarVelocity.normalized * maxRollSpeed;
            boardBody.linearVelocity = cappedPlanar + Vector3.Project(boardBody.linearVelocity, groundNormal);
        }
    }

    void ApplyFriction() {
        if (!boardBody)
            return;
        if (!hasGroundContact)
            return;

        Vector3 planarVelocity = Vector3.ProjectOnPlane(boardBody.linearVelocity, groundNormal);
        if (planarVelocity.sqrMagnitude < 0.001f)
            return;

        Vector3 forward = Vector3.ProjectOnPlane(boardBody.transform.forward, groundNormal);
        if (forward.sqrMagnitude > 0.0001f)
            forward.Normalize();
        else
            forward = planarVelocity.normalized;

        float forwardSpeed = Vector3.Dot(planarVelocity, forward);
        Vector3 forwardVelocity = forward * forwardSpeed;
        Vector3 resistance = -forwardVelocity * rollingResistance;
        boardBody.AddForce(resistance, ForceMode.Acceleration);
    }
    void UpdateLeanVisual(float dt) {
        if (!boardVisualRoot)
            return;

        if (!trickPoseActive) {
            visualBaseRotation = boardVisualRoot.localRotation;
            if (allowTrickPositionOffset)
                visualBasePosition = boardVisualRoot.localPosition;
        }

        Quaternion targetRotation = visualBaseRotation;
        Vector3 targetPosition = allowTrickPositionOffset ? visualBasePosition : boardVisualRoot.localPosition;

        if (trickPoseActive) {
            targetRotation *= trickRotationOffset;
            if (allowTrickPositionOffset)
                targetPosition = visualBasePosition + trickPositionOffset;
        } else {
            if (boardVisualRoot == transform)
                return;

            float targetLean = -moveInput.x * leanAngle;
            targetRotation *= Quaternion.AngleAxis(targetLean, Vector3.forward);
        }

        float t = DampedLerp(leanResponse, dt);
        boardVisualRoot.localRotation = Quaternion.Slerp(boardVisualRoot.localRotation, targetRotation, t);

        if (allowTrickPositionOffset)
            boardVisualRoot.localPosition = Vector3.Lerp(boardVisualRoot.localPosition, targetPosition, t);
    }

    void UpdateKinematicState()
    {
        velocity = boardBody.linearVelocity;
        Vector3 planarVelocity = Vector3.ProjectOnPlane(velocity, groundNormal);
        speed = planarVelocity.magnitude;
    }

    public void ApplyTrickPose(Quaternion rotationOffset, Vector3 positionOffset) {
        trickPoseActive = true;
        trickRotationOffset = rotationOffset;
        trickPositionOffset = allowTrickPositionOffset ? positionOffset : Vector3.zero;
    }

    public void ClearTrickPose() {
        trickPoseActive = false;
        trickRotationOffset = Quaternion.identity;
        trickPositionOffset = Vector3.zero;
    }

    static float DampedLerp(float speed, float dt) => 1f - Mathf.Exp(-Mathf.Max(speed, 0f) * dt);

    public void ClearPushQueue() {
        pushQueued = false;
        pushRequest = false;
    }
}
