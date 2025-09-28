using System.Collections.Generic;
using UnityEngine;

public sealed class FlickItRecognizer {
    readonly FlickItSettings S;
    readonly List<Vector2> samples = new(32);

    Vector2 prevRS;
    bool collecting;
    float maxSpeed;
    float pathTravel;
    float collectTime;

    public FlickItRecognizer(FlickItSettings settings) { S = settings; }

    public void Reset() {
        collecting = false;
        samples.Clear();
        prevRS = Vector2.zero;
        maxSpeed = 0f;
        pathTravel = 0f;
        collectTime = 0f;
    }

    public bool Tick(Vector2 rs, float dt, out TrickId trick, out bool nollie) {
        trick = TrickId.None;
        nollie = false;

        float mag = rs.magnitude;
        if (!collecting) {
            if (mag > S.deadzone) Begin(rs);
        } else {
            collectTime += dt;

            Vector2 delta = rs - prevRS;
            float step = delta.magnitude;
            pathTravel += step;
            if (dt > 0f) {
                float speed = step / dt;
                if (speed > maxSpeed) maxSpeed = speed;
            }

            if (samples.Count == 0 || Vector2.Distance(samples[^1], rs) > 0.01f)
                samples.Add(rs);

            float releaseThreshold = S.deadzone * 0.6f;
            if (mag <= releaseThreshold || collectTime >= 0.45f) {
                if (samples.Count == 0 || Vector2.Distance(samples[^1], rs) > 0.002f)
                    samples.Add(rs);
                bool success = Evaluate(out trick, out nollie);
                Reset();
                prevRS = rs;
                return success;
            }
        }

        prevRS = rs;
        return false;
    }

    void Begin(Vector2 rs) {
        collecting = true;
        collectTime = 0f;
        samples.Clear();
        samples.Add(rs);
        prevRS = rs;
        maxSpeed = 0f;
        pathTravel = 0f;
    }

    bool Evaluate(out TrickId id, out bool nollie) {
        id = TrickId.None;
        nollie = false;

        if (samples.Count < 3)
            return false;
        if (maxSpeed < S.minFlickSpeed || pathTravel < S.minPathTravel)
            return false;

        float minX = float.MaxValue, maxX = float.MinValue;
        float minY = float.MaxValue, maxY = float.MinValue;
        int minXIndex = 0, maxXIndex = 0, minYIndex = 0, maxYIndex = 0;

        for (int i = 0; i < samples.Count; i++) {
            Vector2 p = samples[i];
            if (p.x < minX) { minX = p.x; minXIndex = i; }
            if (p.x > maxX) { maxX = p.x; maxXIndex = i; }
            if (p.y < minY) { minY = p.y; minYIndex = i; }
            if (p.y > maxY) { maxY = p.y; maxYIndex = i; }
        }

        float verticalSpan = maxY - minY;
        float horizontalSpan = maxX - minX;

        const float verticalThreshold = 0.4f;
        const float horizontalThreshold = 0.28f;
        const float comboThreshold = horizontalThreshold * 1.25f;

        bool downThenUp = minYIndex < maxYIndex;
        bool upThenDown = maxYIndex < minYIndex;
        bool hasPos = maxX >= horizontalThreshold;
        bool hasNeg = minX <= -horizontalThreshold;
        bool strongPos = maxX >= comboThreshold;
        bool strongNeg = minX <= -comboThreshold;

        if (downThenUp && verticalSpan >= verticalThreshold && -minY >= 0.2f && maxY >= 0.18f) {
            if (strongPos && strongNeg) {
                if (minXIndex < maxXIndex) {
                    id = TrickId.TreFlip;
                    return true;
                }
                if (maxXIndex < minXIndex) {
                    id = TrickId.Laserflip;
                    return true;
                }
            }

            if (hasPos && !hasNeg) {
                id = TrickId.Kickflip;
                return true;
            }
            if (hasNeg && !hasPos) {
                id = TrickId.Heelflip;
                return true;
            }

            id = TrickId.Ollie;
            return true;
        }

        if (upThenDown && verticalSpan >= verticalThreshold && maxY >= 0.2f) {
            if (!hasPos && !hasNeg) {
                id = TrickId.Nollie;
                nollie = true;
                return true;
            }
        }

        if (horizontalSpan >= horizontalThreshold && verticalSpan < verticalThreshold * 0.6f) {
            if (hasNeg && !hasPos) {
                id = TrickId.ShuvitFS;
                return true;
            }
            if (hasPos && !hasNeg) {
                id = TrickId.ShuvitBS;
                return true;
            }
        }

        return false;
    }
}
