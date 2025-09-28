using UnityEngine;

[CreateAssetMenu(menuName = "Skate/FlickIt Settings")]
public class FlickItSettings : ScriptableObject {
    [Header("Zones")]
    [Range(0f,1f)] public float deadzone = 0.18f;
    [Range(0f,1f)] public float halfTilt = 0.45f; // Manuals (Down/Up half)

    [Header("Flick")]
    public float minFlickSpeed = 1.4f;   // RS speed threshold
    public float minPathTravel = 0.12f;  // normalized travel
    [Range(0f,45f)] public float angleTolerance = 25f;

    [Header("Buffering")]
    public float coyoteTime = 0.12f;     // after takeoff
    public float lateFlipWindow = 0.18f; // in-air flips after pop
}
