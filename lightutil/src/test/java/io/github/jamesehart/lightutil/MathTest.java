package io.github.jamesehart.lightutil;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import io.github.jamesehart.lightutil.math.AngleUtil;
import io.github.jamesehart.lightutil.math.InterpolatingTable;
import io.github.jamesehart.lightutil.math.Polygon2d;

class MathTest {
    private static final double EPS = 1e-9;

    @Test
    void wrapsAngles() {
        assertEquals(-Math.PI, AngleUtil.wrapPi(Math.PI), EPS);
        assertEquals(-Math.PI / 2, AngleUtil.wrapPi(3 * Math.PI / 2), EPS);
        assertEquals(0.5, AngleUtil.wrapPi(0.5 + 4 * Math.PI), EPS);
        assertEquals(3 * Math.PI / 2, AngleUtil.wrap2Pi(-Math.PI / 2), EPS);
        assertEquals(0, AngleUtil.wrap2Pi(2 * Math.PI), EPS);
    }

    @Test
    void shortestDeltaCrossesPi() {
        assertEquals(0.2, AngleUtil.shortestDelta(Math.PI - 0.1, -Math.PI + 0.1), EPS);
        assertEquals(-0.2, AngleUtil.shortestDelta(-Math.PI + 0.1, Math.PI - 0.1), EPS);
    }

    @Test
    void placeInRangeTakesShortestLegalPath() {
        // A turret that can turn ±270°: from +170°, -170° is reached by going on to +190°.
        double min = Math.toRadians(-270);
        double max = Math.toRadians(270);
        assertEquals(Math.toRadians(190),
                AngleUtil.placeInRange(Math.toRadians(-170), Math.toRadians(170), min, max), EPS);
        // At 260°, 280° is past the limit, so it unwinds the long way to -80° instead.
        assertEquals(Math.toRadians(-80),
                AngleUtil.placeInRange(Math.toRadians(-80), Math.toRadians(260), min, max), EPS);
        // A ±90° mechanism can't reach 170°, so it goes to the closest limit.
        assertEquals(Math.PI / 2,
                AngleUtil.placeInRange(Math.toRadians(170), 0, -Math.PI / 2, Math.PI / 2), EPS);
    }

    @Test
    void interpolatesAndClamps() {
        InterpolatingTable table = new InterpolatingTable().add(2, 3000, 0.4).add(4, 3800, 0.6);
        assertArrayEquals(new double[] {3400, 0.5}, table.get(3), EPS);
        assertArrayEquals(new double[] {3000, 0.4}, table.get(0), EPS);
        assertArrayEquals(new double[] {3800, 0.6}, table.get(10), EPS);
        assertEquals(3800, table.get(4, 0), EPS);
        assertThrows(IllegalArgumentException.class, () -> table.add(5, 1));
    }

    @Test
    void polygonContainsConcave() {
        // An L shape.
        Polygon2d l = new Polygon2d(
                new Translation2d(0, 0), new Translation2d(2, 0), new Translation2d(2, 1),
                new Translation2d(1, 1), new Translation2d(1, 2), new Translation2d(0, 2));
        assertTrue(l.contains(new Translation2d(0.5, 1.5)));
        assertTrue(l.contains(new Translation2d(1.5, 0.5)));
        assertFalse(l.contains(new Translation2d(1.5, 1.5)));
        assertFalse(l.contains(new Translation2d(-1, 0.5)));

        Polygon2d shifted = l.map(t -> t.plus(new Translation2d(10, 0)));
        assertTrue(shifted.contains(new Translation2d(10.5, 1.5)));
    }
}
