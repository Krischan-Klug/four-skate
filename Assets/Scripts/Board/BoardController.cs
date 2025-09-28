using UnityEngine;

public sealed class BoardController : MonoBehaviour {
    [Header("Input & Tricks")]
    [SerializeField] FlickItSettings flickSettings;
    [SerializeField] UnityInputAdapter input;

    FlickItRecognizer recognizer;
    TrickEvent lastTrick;
    float timeSinceLastTrick;

    bool pushQueued;

    public Vector2 MoveInput => input ? input.MoveLS : Vector2.zero;
    public Vector2 TrickInput => input ? input.TrickRS : Vector2.zero;
    public bool PushPressed => input && input.PushPressed;
    public float GrabLeft => input ? input.GrabLeft : 0f;
    public float GrabRight => input ? input.GrabRight : 0f;

    public bool IsGrounded => true;
    public bool InAir => false;
    public float AirTime => 0f;
    public float Speed => 0f;
    public bool PushQueued => pushQueued;
    public bool PopReady => true;
    public TrickId CurrentQueuedTrick => lastTrick.id;
    public Vector3 GroundNormal => Vector3.up;
    public Vector3 Velocity => Vector3.zero;

    public TrickEvent LastTrick => lastTrick;
    public float TimeSinceLastTrick => timeSinceLastTrick;

    public event System.Action<TrickEvent> TrickPerformed;

    void Awake() {
        if (flickSettings)
            recognizer = new FlickItRecognizer(flickSettings);
        else
            Debug.LogWarning("BoardController: FlickItSettings missing.", this);
    }

    void Update() {
        float dt = Time.deltaTime;
        timeSinceLastTrick += dt;

        if (input && recognizer != null && recognizer.Tick(input.TrickRS, dt, out var trick, out bool nollie) && trick != TrickId.None) {
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

        if (PushPressed)
            pushQueued = true;
    }

    public void ClearPushQueue() => pushQueued = false;
}
