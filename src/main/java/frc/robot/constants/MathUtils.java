package frc.robot.constants;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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

    public static Translation2d findClosestPointOnTravelEdge(Pose2d robotPose, double translationX, double translationY, Translation2d[] borderPoints) {
        Translation2d robotPosition = robotPose.getTranslation();

        // Compute the unit direction of travel
        double velocityMagnitude = Math.hypot(translationX, translationY);
        if (velocityMagnitude == 0) {
            return null; // Robot is not moving
        }
        double dirX = translationX / velocityMagnitude;
        double dirY = translationY / velocityMagnitude;

        Translation2d closestPoint = null;
        double minAngle = Double.MAX_VALUE;
        Translation2d bestP1 = null, bestP2 = null;

        // Find the polygon edge the robot is traveling toward
        for (int i = 0; i < borderPoints.length; i++) {
            Translation2d p1 = borderPoints[i];
            Translation2d p2 = borderPoints[(i + 1) % borderPoints.length]; // Wrap around

            // Compute vector from robot to the line segment
            double edgeX = p2.getX() - p1.getX();
            double edgeY = p2.getY() - p1.getY();
            double toEdgeX = (p1.getX() + p2.getX()) / 2 - robotPosition.getX();
            double toEdgeY = (p1.getY() + p2.getY()) / 2 - robotPosition.getY();

            // Compute angle between movement vector and edge direction
            double dotProduct = (toEdgeX * dirX + toEdgeY * dirY);
            double magnitudeA = Math.hypot(toEdgeX, toEdgeY);
            double magnitudeB = Math.hypot(dirX, dirY);
            double angle = Math.acos(dotProduct / (magnitudeA * magnitudeB));

            if (dotProduct > 0 && angle < minAngle) { // Edge is in the direction of travel
                minAngle = angle;
                bestP1 = p1;
                bestP2 = p2;
            }
        }

        // If a valid edge is found, get the closest point on it
        if (bestP1 != null && bestP2 != null) {
            closestPoint = closestPointOnSegment(bestP1, bestP2, robotPosition);
        }

        return closestPoint;
    }

    // Computes the closest point on a line segment (p1, p2) to a given point
    private static Translation2d closestPointOnSegment(Translation2d p1, Translation2d p2, Translation2d point) {
        double segmentLengthSquared = Math.pow(p2.getX() - p1.getX(), 2) + Math.pow(p2.getY() - p1.getY(), 2);
        if (segmentLengthSquared == 0) return p1; // p1 and p2 are the same point

        // Project point onto the segment, clamping t to [0,1] to stay within segment bounds
        double t = ((point.getX() - p1.getX()) * (p2.getX() - p1.getX()) +
                    (point.getY() - p1.getY()) * (p2.getY() - p1.getY())) / segmentLengthSquared;
        t = Math.max(0, Math.min(1, t)); // Clamp to segment

        // Compute the closest point
        return new Translation2d(
            p1.getX() + t * (p2.getX() - p1.getX()),
            p1.getY() + t * (p2.getY() - p1.getY())
        );
    }

}
