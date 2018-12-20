package com.spartronics4915.lib.lidar;

import com.spartronics4915.lib.geometry.Translation2d;
import com.spartronics4915.lib.geometry.Pose2d;

/**
 * Represents a single point from the LIDAR sensor. This consists of
 * an angle, distance, and timestamp.
 */
class LidarPoint
{
    public static final double MM_TO_IN = 1 / 25.4; // 1 inch = 25.4 millimeters

    public final double timestamp;
    public final double angle;
    public final double distance;

    public LidarPoint(double timestamp, double angle, double distance)
    {
        this.timestamp = timestamp;
        this.angle = angle;
        this.distance = distance * MM_TO_IN;
    }

    /**
     * Convert this point into a {@link Translation2d} in cartesian (x, y)
     * coordinates. The point's timestamp is used along with the {@link RobotState}
     * to take into account the robot's pose at the time the point was detected.
     * 
     * @param robotPose: optional robotPose, used to transform lidar points
     *  to field coordinates.  If not provided, we're operating in "relative"
     *  mode.  If provided, robotPose should include the robotToLidar 
     *  transform.
     */
    public Translation2d toCartesian(Pose2d robotPose)
    {
        // convert the polar coords to cartesian coords
        double radians = Math.toRadians(this.angle);
        Translation2d x2d = new Translation2d(Math.cos(radians) * this.distance, 
                                              Math.sin(radians) * this.distance);
        if(robotPose != null)
        {
            return robotPose.transformBy(Pose2d.fromTranslation(x2d)).getTranslation();
        }
        else
            return x2d;
    }
}
