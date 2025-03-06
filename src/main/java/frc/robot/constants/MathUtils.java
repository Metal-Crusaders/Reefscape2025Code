package frc.robot.constants;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class MathUtils {

    public static boolean withinTolerance(Rotation2d value, Rotation2d tolerance) {
        return withinTolerance(value.getDegrees(), tolerance.getDegrees());
    }

    public static boolean withinTolerance(double value, double tolerance) {
        return Math.abs(value) <= Math.abs(tolerance);
    }

    public static Pose2d findClosestTarget(Pose2d current, Pose2d[] targets) {
        if (current == null) {
            return null;
        }
        if (targets == null) {
            throw new IllegalArgumentException("Target list cannot be null or empty.");
        }

        Pose2d closest = null;
        double minDistance = Double.MAX_VALUE;

        for (Pose2d target : targets) {
            double distance = current.getTranslation().getDistance(target.getTranslation());
            if (distance <= minDistance) {
                minDistance = distance;
                closest = target;
            }
        }

        return closest;
    }

    public static int findClosestIdx(Pose2d current, Pose2d[] targets) {
        if (current == null) {
            return 0;
        }
        if (targets == null) {
            throw new IllegalArgumentException("Target list cannot be null or empty.");
        }

        int closest = 0;
        double minDistance = Double.MAX_VALUE;

        for (int i = 0; i < targets.length; i++) {
            Pose2d target = targets[i];
            double distance = current.getTranslation().getDistance(target.getTranslation());
            if (distance <= minDistance) {
                minDistance = distance;
                closest = i;
            }
        }

        return closest;
    }

    public static boolean withinField(Pose2d point, Pose2d[] polygon) {
        int n = polygon.length;
        if (n < 3) return false; // Not a valid polygon

        boolean inside = false;
        double px = point.getX(), py = point.getY();

        for (int i = 0, j = n - 1; i < n; j = i++) {
            Pose2d pi = polygon[i];
            Pose2d pj = polygon[j];

            double xi = pi.getX(), yi = pi.getY();
            double xj = pj.getX(), yj = pj.getY();

            // Check if the point's y is within the range of the segment's y values
            if ((yi > py) != (yj > py)) {
                // Compute the x coordinate of the intersection
                double xIntersect = xi + (py - yi) * (xj - xi) / (yj - yi);
                
                // If the point's x is to the left of the intersection, toggle inside flag
                if (px < xIntersect) {
                    inside = !inside;
                }
            }
        }

        return inside;
    }

    public static Pose2d getRayIntersection(Pose2d pose, Pose2d[] polygon) {
        double px = pose.getX(), py = pose.getY();
        double dx = pose.getRotation().getCos(), dy = pose.getRotation().getSin();

        Pose2d closestIntersection = null;
        double minDistance = Double.POSITIVE_INFINITY;

        int n = polygon.length;
        for (int i = 0, j = n - 1; i < n; j = i++) {
            Pose2d p1 = polygon[j];
            Pose2d p2 = polygon[i];

            Pose2d intersection = getLineIntersection(px, py, dx, dy, p1.getX(), p1.getY(), p2.getX(), p2.getY());

            if (intersection != null) {
                double distance = Math.hypot(intersection.getX() - px, intersection.getY() - py);
                if (distance < minDistance) {
                    minDistance = distance;
                    closestIntersection = intersection;
                }
            }
        }
        return closestIntersection;
    }

    /**
     * Computes the intersection point between a ray (px, py) + t*(dx, dy)
     * and a line segment (x1, y1) -> (x2, y2).
     * Returns the intersection point or null if there is no intersection.
     */
    private static Pose2d getLineIntersection(double px, double py, double dx, double dy,
                                              double x1, double y1, double x2, double y2) {
        double sx = x2 - x1, sy = y2 - y1; // Segment direction vector
        double denom = dx * sy - dy * sx;

        if (Math.abs(denom) < 1e-9) return null; // Lines are parallel (no intersection)

        double t = ((x1 - px) * sy - (y1 - py) * sx) / denom;
        double u = ((x1 - px) * dy - (y1 - py) * dx) / denom;

        if (t >= 0 && u >= 0 && u <= 1) {
            return new Pose2d(px + t * dx, py + t * dy, new Rotation2d());
        }
        return null; // No valid intersection
    }
    
}
