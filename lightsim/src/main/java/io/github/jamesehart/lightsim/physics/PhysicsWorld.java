package io.github.jamesehart.lightsim.physics;

import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.world.World;

/**
 * The top-down 2D field: a dyn4j world with no gravity (we look down on the field), walls around the
 * perimeter, and optional static obstacles. The origin is the blue-alliance corner, matching WPILib
 * field coordinates.
 */
public final class PhysicsWorld {
    private static final double WALL_THICKNESS = 1.0;

    private final World<Body> world = new World<>();
    private final double fieldLength;
    private final double fieldWidth;

    /**
     * @param fieldLengthM field length (x) in meters
     * @param fieldWidthM field width (y) in meters
     */
    public PhysicsWorld(double fieldLengthM, double fieldWidthM) {
        this.fieldLength = fieldLengthM;
        this.fieldWidth = fieldWidthM;
        world.setGravity(0, 0);
        world.getSettings().setAtRestDetectionEnabled(false);

        double t = WALL_THICKNESS;
        addObstacle(fieldLengthM / 2, -t / 2, 0, fieldLengthM + 2 * t, t);
        addObstacle(fieldLengthM / 2, fieldWidthM + t / 2, 0, fieldLengthM + 2 * t, t);
        addObstacle(-t / 2, fieldWidthM / 2, 0, t, fieldWidthM);
        addObstacle(fieldLengthM + t / 2, fieldWidthM / 2, 0, t, fieldWidthM);
    }

    /**
     * Adds an immovable rectangle.
     *
     * @param centerX center x in meters
     * @param centerY center y in meters
     * @param rotationRad rotation, counter-clockwise
     * @param lengthM size along x before rotation
     * @param widthM size along y before rotation
     */
    public void addObstacle(double centerX, double centerY, double rotationRad, double lengthM, double widthM) {
        Body obstacle = new Body();
        BodyFixture fixture = obstacle.addFixture(Geometry.createRectangle(lengthM, widthM));
        fixture.setFriction(0.5);
        fixture.setRestitution(0.1);
        obstacle.setMass(MassType.INFINITE);
        obstacle.rotate(rotationRad);
        obstacle.translate(centerX, centerY);
        world.addBody(obstacle);
    }

    public double getFieldLength() {
        return fieldLength;
    }

    public double getFieldWidth() {
        return fieldWidth;
    }

    World<Body> getWorld() {
        return world;
    }
}
