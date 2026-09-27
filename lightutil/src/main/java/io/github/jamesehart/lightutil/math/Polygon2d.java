package io.github.jamesehart.lightutil.math;

import java.util.List;
import java.util.function.UnaryOperator;

import edu.wpi.first.math.geometry.Translation2d;

/** A polygon on the field, for zones like "our half" or "keep out". Concave shapes work too. */
public final class Polygon2d {
    private final Translation2d[] vertices;

    /**
     * Creates a polygon from its corners, in order (either direction).
     *
     * @param vertices at least three corners
     */
    public Polygon2d(Translation2d... vertices) {
        if (vertices.length < 3) throw new IllegalArgumentException("A polygon needs at least 3 vertices");
        this.vertices = vertices.clone();
    }

    /** Creates a polygon from its corners, in order. */
    public Polygon2d(List<Translation2d> vertices) {
        this(vertices.toArray(new Translation2d[0]));
    }

    /** Creates an axis-aligned rectangle from two opposite corners. */
    public static Polygon2d rectangle(Translation2d corner, Translation2d opposite) {
        return new Polygon2d(
                corner,
                new Translation2d(opposite.getX(), corner.getY()),
                opposite,
                new Translation2d(corner.getX(), opposite.getY()));
    }

    /** Returns true if the point is inside the polygon (points exactly on an edge may go either way). */
    public boolean contains(Translation2d point) {
        double x = point.getX();
        double y = point.getY();
        boolean inside = false;
        for (int i = 0, j = vertices.length - 1; i < vertices.length; j = i++) {
            double xi = vertices[i].getX();
            double yi = vertices[i].getY();
            double xj = vertices[j].getX();
            double yj = vertices[j].getY();
            if ((yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) {
                inside = !inside;
            }
        }
        return inside;
    }

    /** Returns a copy with every vertex transformed, e.g. {@code polygon.map(AllianceFlip::flip)}. */
    public Polygon2d map(UnaryOperator<Translation2d> transform) {
        Translation2d[] mapped = new Translation2d[vertices.length];
        for (int i = 0; i < vertices.length; i++) {
            mapped[i] = transform.apply(vertices[i]);
        }
        return new Polygon2d(mapped);
    }

    /** The corners, in order. */
    public Translation2d[] getVertices() {
        return vertices.clone();
    }
}
