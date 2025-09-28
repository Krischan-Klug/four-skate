using System.Text;
using UnityEngine;
using UnityEngine.UI;

public sealed class SkateDebugHUD : MonoBehaviour {
    [Header("References")]
    [SerializeField] BoardController board;
    [SerializeField] UnityInputAdapter input;

    [Header("Left Stick UI")]
    [SerializeField] RectTransform leftStickArea;
    [SerializeField] RectTransform leftStickHandle;

    [Header("Right Stick UI")]
    [SerializeField] RectTransform rightStickArea;
    [SerializeField] RectTransform rightStickHandle;

    [Header("Info Output")]
    [SerializeField] Text infoText;

    readonly StringBuilder sb = new StringBuilder(256);

    void LateUpdate() {
        if (!board || !input)
            return;

        UpdateStick(leftStickArea, leftStickHandle, input.MoveLS);
        UpdateStick(rightStickArea, rightStickHandle, input.TrickRS);
        UpdateInfo();
    }

    void UpdateStick(RectTransform area, RectTransform handle, Vector2 value) {
        if (!area || !handle)
            return;

        Vector2 halfSize = area.rect.size * 0.5f;
        Vector2 clamped = Vector2.ClampMagnitude(value, 1f);
        handle.anchoredPosition = new Vector2(clamped.x * halfSize.x, clamped.y * halfSize.y);
    }

    void UpdateInfo() {
        if (!infoText)
            return;

        sb.Clear();
        sb.AppendLine($"Grounded: {board.IsGrounded}");
        sb.AppendLine($"In Air: {board.InAir}");
        sb.AppendLine($"Air Time: {board.AirTime:F2}s");
        sb.AppendLine($"Speed: {board.Speed:F2} m/s");
        sb.AppendLine($"Pop Ready: {board.PopReady}");
        sb.AppendLine($"Push Queued: {board.PushQueued}");
        sb.AppendLine($"Queued Trick: {board.CurrentQueuedTrick}");
        sb.AppendLine($"Velocity: {board.Velocity}");
        sb.AppendLine($"Ground Normal: {board.GroundNormal}");

        infoText.text = sb.ToString();
    }
}
