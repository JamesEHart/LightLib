package io.github.jamesehart.lightutil;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import io.github.jamesehart.lightutil.season2026.HubShift;
import io.github.jamesehart.lightutil.vision.GamePieceTracker;

class TrackingAndHubTest {
    private static final double EPS = 1e-6;

    @Test
    void trackerMergesConfirmsAndForgets() {
        GamePieceTracker tracker = new GamePieceTracker();
        tracker.add(new Translation2d(2, 2), 0);
        tracker.add(new Translation2d(5, 5), 0);
        assertEquals(0, tracker.getPieces().size(), "seen once each: not confirmed yet");

        tracker.add(new Translation2d(2.2, 2), 0.1);
        assertEquals(1, tracker.getPieces().size());
        assertEquals(2.1, tracker.getPieces().get(0).position().getX(), EPS);

        tracker.add(new Translation2d(5, 5), 0.5);
        Translation2d closest = tracker.closest(new Pose2d(6, 6, Rotation2d.kZero)).orElseThrow();
        assertEquals(new Translation2d(5, 5), closest);
        assertEquals(new Translation2d(2.1, 2),
                tracker.closest(new Pose2d(6, 6, Rotation2d.kZero), p -> p.getX() < 4).orElseThrow());

        tracker.removeOlderThan(1.2);   // (2.1, 2) last seen at 0.1, (5, 5) at 0.5
        assertEquals(1, tracker.getPieces().size());
        tracker.removeNear(new Translation2d(5, 5.1), 0.3);
        assertTrue(tracker.closest(Pose2d.kZero).isEmpty());
    }

    @Test
    void hubFollowsShifts() {
        // Red scored more in auto ("R"), so red's hub goes inactive first.
        assertTrue(HubShift.isActive(true, "R", 135), "transition shift");
        assertFalse(HubShift.isActive(true, "R", 120), "shift 1");
        assertTrue(HubShift.isActive(true, "R", 100), "shift 2");
        assertFalse(HubShift.isActive(true, "R", 60), "shift 3");
        assertTrue(HubShift.isActive(true, "R", 40), "shift 4");
        assertTrue(HubShift.isActive(true, "R", 20), "endgame");
        assertTrue(HubShift.isActive(false, "R", 120), "blue gets shift 1");
        assertFalse(HubShift.isActive(false, "R", 100));
        assertTrue(HubShift.isActive(true, "", 120), "no game data yet: assume active");
        // Exactly on a boundary counts as the next shift, like WPILib's example.
        assertTrue(HubShift.isActive(true, "R", 105));
    }

    @Test
    void hubTimeUntilChange() {
        assertEquals(15, HubShift.timeUntilChange(true, "R", 120), EPS);
        // Blue is active through the transition and shift 1, so it changes at 1:45.
        assertEquals(30, HubShift.timeUntilChange(false, "R", 135), EPS);
        assertEquals(Double.POSITIVE_INFINITY, HubShift.timeUntilChange(true, "R", 20));
        assertEquals(Double.POSITIVE_INFINITY, HubShift.timeUntilChange(true, "", 120));
    }
}
