using UnityEngine;

[System.Serializable]
public sealed class TrickDefinition {
    [Tooltip("Identifier from TrickId enum that this definition responds to.")]
    public TrickId trickId = TrickId.None;

    [Tooltip("Total duration of the trick playback in seconds.")]
    [Min(0.05f)] public float duration = 0.6f;

    [Tooltip("Local rotation applied over the trick (degrees).")]
    public Vector3 rotationDegrees = Vector3.zero;

    [Tooltip("Curve controlling how rotationDegrees is applied over time (0-1).")]
    public AnimationCurve rotationCurve = AnimationCurve.EaseInOut(0f, 0f, 1f, 1f);

    [Tooltip("Additional local position offset (meters) applied using positionCurve.")]
    public Vector3 positionOffset = Vector3.zero;

    [Tooltip("Curve controlling how positionOffset is applied over time (0-1).")]
    public AnimationCurve positionCurve = AnimationCurve.EaseInOut(0f, 0f, 1f, 0f);

    [Tooltip("Optional angular velocity impulse applied to the rigidbody at trick start (deg/s).")]
    public Vector3 angularVelocity = Vector3.zero;

    [Tooltip("Optional velocity change applied to the rigidbody at trick start (m/s).")]
    public Vector3 popImpulse = Vector3.zero;

    public TrickDefinition Clone() => (TrickDefinition)MemberwiseClone();
}


