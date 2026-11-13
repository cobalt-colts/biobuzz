package org.firstinspires.ftc.teamcode.robot.util;

/** Conservative evidence of target absence, not proof of a physical tip. */
public final class TipConfirmation {
    private static final int REQUIRED_FRAMES = 8;
    private static final long MIN_SPAN_NS = 600_000_000L;
    private static final long MAX_GAP_NS = 300_000_000L;
    private int frames;
    private long firstFrame;
    private long lastFrame;
    private boolean sawAlliance;

    public void reset() {
        frames = 0;
        firstFrame = lastFrame = 0;
        sawAlliance = false;
    }

    public void observe(boolean allianceVisible, long now) {
        // Any alliance sighting vetoes this entire observation window.
        sawAlliance |= allianceVisible;
        if (frames > 0 && now <= lastFrame) return;
        if (frames > 0 && now - lastFrame > MAX_GAP_NS) frames = 0;
        if (frames == 0) firstFrame = now;
        lastFrame = now;
        frames++;
    }

    public boolean isConfirmed(long now) {
        return !sawAlliance && frames >= REQUIRED_FRAMES
                && lastFrame - firstFrame >= MIN_SPAN_NS
                && now >= lastFrame && now - lastFrame <= MAX_GAP_NS;
    }
}
