using UnityEngine;
using UnityEngine.InputSystem;

public interface IRidingInput {
    Vector2 MoveLS { get; }
    Vector2 TrickRS { get; }
    bool PushPressed { get; }
    float GrabLeft { get; }   // LT
    float GrabRight { get; }  // RT
}

public sealed class UnityInputAdapter : MonoBehaviour, IRidingInput {
    [SerializeField] InputActionProperty moveLS;
    [SerializeField] InputActionProperty trickRS;
    [SerializeField] InputActionProperty push;
    [SerializeField] InputActionProperty grabLeft;
    [SerializeField] InputActionProperty grabRight;

    public Vector2 MoveLS => moveLS.action?.ReadValue<Vector2>() ?? Vector2.zero;
    public Vector2 TrickRS => trickRS.action?.ReadValue<Vector2>() ?? Vector2.zero;
    public bool PushPressed => push.action != null && push.action.WasPressedThisFrame();
    public float GrabLeft => grabLeft.action?.ReadValue<float>() ?? 0f;
    public float GrabRight => grabRight.action?.ReadValue<float>() ?? 0f;

    void OnEnable() {
        moveLS.action?.Enable(); trickRS.action?.Enable(); push.action?.Enable();
        grabLeft.action?.Enable(); grabRight.action?.Enable();
    }
    void OnDisable() {
        moveLS.action?.Disable(); trickRS.action?.Disable(); push.action?.Disable();
        grabLeft.action?.Disable(); grabRight.action?.Disable();
    }
}
