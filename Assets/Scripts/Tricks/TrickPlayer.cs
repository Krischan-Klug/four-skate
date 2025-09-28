using System;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class TrickPlayer : MonoBehaviour {
    [Header("References")]
    [SerializeField] BoardController board;
    [SerializeField] Rigidbody boardBodyOverride;

    [Header("Trick Library")]
    [SerializeField] TrickDefinition[] tricks = Array.Empty<TrickDefinition>();

    [Header("Physics Tuning")]
    [SerializeField] bool applyAngularVelocity = true;
    [SerializeField] float angularVelocityScale = 1f;

    Rigidbody boardBody;
    TrickPlayback active;

    void Reset() {
        tricks = new[] {
            new TrickDefinition {
                trickId = TrickId.Ollie,
                duration = 0.45f,
                rotationDegrees = new Vector3(-12f, 0f, 0f),
                rotationCurve = AnimationCurve.EaseInOut(0f, 0f, 1f, 1f),
                positionOffset = new Vector3(0f, 0.25f, 0f),
                positionCurve = new AnimationCurve(
                    new Keyframe(0f, 0f, 0f, 4f),
                    new Keyframe(0.4f, 1f, 0f, 0f),
                    new Keyframe(1f, 0f, -4f, 0f)
                ),
                popImpulse = new Vector3(0f, 2.6f, 0.45f)
            },
            new TrickDefinition {
                trickId = TrickId.Kickflip,
                duration = 0.65f,
                rotationDegrees = new Vector3(0f, 0f, 360f),
                rotationCurve = AnimationCurve.EaseInOut(0f, 0f, 1f, 1f),
                positionOffset = new Vector3(0f, 0.2f, 0f),
                positionCurve = new AnimationCurve(
                    new Keyframe(0f, 0f, 0f, 3.5f),
                    new Keyframe(0.35f, 1f, 0f, 0f),
                    new Keyframe(1f, 0f, -3.5f, 0f)
                ),
                angularVelocity = new Vector3(0f, 0f, 420f),
                popImpulse = new Vector3(0f, 2.8f, 0.6f)
            },
            new TrickDefinition {
                trickId = TrickId.Heelflip,
                duration = 0.65f,
                rotationDegrees = new Vector3(0f, 0f, -360f),
                rotationCurve = AnimationCurve.EaseInOut(0f, 0f, 1f, 1f),
                positionOffset = new Vector3(0f, 0.2f, 0f),
                positionCurve = new AnimationCurve(
                    new Keyframe(0f, 0f, 0f, 3.5f),
                    new Keyframe(0.35f, 1f, 0f, 0f),
                    new Keyframe(1f, 0f, -3.5f, 0f)
                ),
                angularVelocity = new Vector3(0f, 0f, -420f),
                popImpulse = new Vector3(0f, 2.8f, 0.6f)
            }
        };
    }

    void Awake() {
        if (!board)
            board = GetComponent<BoardController>();

        boardBody = boardBodyOverride ? boardBodyOverride : board ? board.BoardBody : null;
    }

    void OnEnable() {
        if (board != null)
            board.TrickPerformed += HandleTrickEvent;
    }

    void OnDisable() {
        if (board != null)
            board.TrickPerformed -= HandleTrickEvent;
        StopActiveTrick();
    }

    void Update() {
        if (board == null)
            return;

        if (!active.isActive)
            return;

        UpdateActiveTrick();
    }

    void HandleTrickEvent(TrickEvent evt) {
        TrickDefinition definition = FindDefinition(evt.id);
        if (definition == null)
            return;

        StartTrick(definition, evt);
    }

    TrickDefinition FindDefinition(TrickId id) {
        for (int i = 0; i < tricks.Length; i++) {
            if (tricks[i] != null && tricks[i].trickId == id)
                return tricks[i];
        }
        return null;
    }

    void StartTrick(TrickDefinition definition, TrickEvent evt) {
        active.definition = definition;
        active.eventData = evt;
        active.elapsed = 0f;
        active.isActive = true;

        ApplyPhysicsImpulses(definition);
        ApplyPose(0f);
    }

    void UpdateActiveTrick() {
        active.elapsed += Time.deltaTime;
        float duration = Mathf.Max(0.0001f, active.definition.duration);
        float normalized = Mathf.Clamp01(active.elapsed / duration);

        ApplyPose(normalized);

        if (active.elapsed >= active.definition.duration)
            StopActiveTrick();
    }

    void ApplyPose(float normalizedTime) {
        float rotationWeight = Evaluate(active.definition.rotationCurve, normalizedTime);
        Quaternion rotationOffset = Quaternion.Euler(active.definition.rotationDegrees * rotationWeight);

        Vector3 positionOffset = Vector3.zero;
        if (active.definition.positionOffset != Vector3.zero)
            positionOffset = active.definition.positionOffset * Evaluate(active.definition.positionCurve, normalizedTime);

        board.ApplyTrickPose(rotationOffset, positionOffset);
    }

    void StopActiveTrick() {
        if (!active.isActive)
            return;

        active.isActive = false;
        board.ClearTrickPose();
    }

    void ApplyPhysicsImpulses(TrickDefinition definition) {
        if (boardBody == null)
            return;

        if (definition.popImpulse != Vector3.zero)
            boardBody.AddForce(boardBody.transform.TransformVector(definition.popImpulse), ForceMode.VelocityChange);

        if (applyAngularVelocity && definition.angularVelocity != Vector3.zero) {
            Vector3 worldAngular = boardBody.transform.TransformVector(definition.angularVelocity * Mathf.Deg2Rad * angularVelocityScale);
            boardBody.angularVelocity += worldAngular;
        }
    }

    static float Evaluate(AnimationCurve curve, float t) {
        if (curve == null || curve.length == 0)
            return t;
        return curve.Evaluate(t);
    }

    struct TrickPlayback {
        public TrickDefinition definition;
        public TrickEvent eventData;
        public float elapsed;
        public bool isActive;
    }
}
