package com.spartronics4915.frc2019;

import java.util.Arrays;
import java.util.Optional;

import com.spartronics4915.frc2019.Constants.FieldLandmark;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.geometry.Rotation2d;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.RobotStateMap;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class VisionUpdateManager
{
    public static VisionUpdateManager reverseVisionManager = new VisionUpdateManager("Reverse");

    private static final int kRawUpdateNumDoubles = 4; // 2 for x y, 1 for rotation, and 1 for processing time

    private final String kNetworkTablesKey;
    private VisionUpdate mLatestVisionUpdate = null;

    private VisionUpdateManager(String coprocessorID)
    {
        kNetworkTablesKey = "/SmartDashboard/Vision/" + coprocessorID + "/solvePNP";

        NetworkTableInstance.getDefault().addEntryListener(kNetworkTablesKey, (e) -> visionKeyChangedCallback(e),
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    private void visionKeyChangedCallback(EntryNotification entryNotification)
    {
        try
        {
            String rawVisionUpdate = entryNotification.value.getString();
            mLatestVisionUpdate = VisionUpdate.fromRawUpdate(rawVisionUpdate);
        }
        catch (Exception e)
        {
            Logger.exception(e);
            return;
        }

    }

    /**
     * @return the latest vision update _or_ null if there has not been an update yet
     */
    public Optional<VisionUpdate> getLatestVisionUpdate()
    {
        return Optional.ofNullable(mLatestVisionUpdate);
    }

    public static class VisionUpdate
    {

        public final double frameCapturedTime; // Time in seconds where the epoch the boot of the RoboRIO (getFPGATimestamp's epoch)
        public final Pose2d targetRobotRelativePosition; // The target's robot-relative position at frameCapturedTime (x and y in inches)

        private VisionUpdate(double capturedTime, Pose2d targetRelativePosition)
        {
            this.frameCapturedTime = capturedTime;
            this.targetRobotRelativePosition = targetRelativePosition;
        }

        public static VisionUpdate fromRawUpdate(String vu)
        {
            Double[] rawVisionUpdate = Arrays.stream(vu.split(",")).map(Double::parseDouble).toArray(Double[]::new);

            if (rawVisionUpdate.length < kRawUpdateNumDoubles)
                throw new RuntimeException("A vision update must have at least " + kRawUpdateNumDoubles + " doubles in the array. This one has "
                        + rawVisionUpdate.length + ".");

            double frameCapTime = Timer.getFPGATimestamp() - rawVisionUpdate[3];
            Pose2d targetRelativePosition = new Pose2d(rawVisionUpdate[0], rawVisionUpdate[1], Rotation2d.fromDegrees(rawVisionUpdate[2]));

            return new VisionUpdate(frameCapTime, targetRelativePosition);
        }

        public Pose2d getFieldPosition(RobotStateMap stateMap)
        {
            return stateMap.getFieldToVehicle(this.frameCapturedTime).transformBy(targetRobotRelativePosition);
        }

        public Pose2d getCorrectedRobotPose(Pose2d targetFieldPos, RobotStateMap stateMap, double timeToGetAt)
        {
            Pose2d robotPoseRelativeToLastVisionUpdate = stateMap.get(this.frameCapturedTime).pose.inverse().transformBy(stateMap.get(timeToGetAt).pose);
            return this.targetRobotRelativePosition.inverse().transformBy(targetFieldPos).transformBy(robotPoseRelativeToLastVisionUpdate);
        }

        public Pose2d getCorrectedRobotPoseForClosestTarget(RobotStateMap stateMap, double timeToGetAt)
        {
            double smallestTargetDistance = Double.POSITIVE_INFINITY;
            Pose2d closestTargetPose = null;
            Pose2d robotPose = stateMap.getFieldToVehicle(timeToGetAt);

            for (FieldLandmark l : Constants.FieldLandmark.class.getEnumConstants())
            {
                double distance = robotPose.distance(l.fieldPose);
                if (distance < smallestTargetDistance)
                {
                    closestTargetPose = l.fieldPose;
                    smallestTargetDistance = distance;
                }
            }

            if (closestTargetPose == null) throw new RuntimeException("No vision targets are close! Is Constants.kVisionTargetLocations empty?");

            return getCorrectedRobotPose(closestTargetPose, stateMap, timeToGetAt);
        }

    }
}
