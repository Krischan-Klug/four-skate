using UnityEngine;

public enum TrickId {
    None,
    Ollie, Nollie,
    Kickflip, Heelflip,
    ShuvitFS, ShuvitBS,
    TreFlip, Laserflip
}

[System.Serializable]
public struct TrickEvent {
    public TrickId id;
    public bool nollie;
    public int spinDegrees;
    public bool grabLeft;
    public bool grabRight;
}
