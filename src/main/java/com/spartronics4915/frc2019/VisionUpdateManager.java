package com.spartronics4915.frc2019;

import java.util.Arrays;
import java.util.Optional;

import com.spartronics4915.frc2019.Constants.ScorableLandmark;
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

    public static VisionUpdateManager reverseVisionManager = new VisionUpdateManager("Reverse", new Pose2d(-10, 0, Rotation2d.fromDegrees(180)));

    private static final int kRawUpdateNumDoubles = 4; // 2 for x y, 1 for rotation, and 1 for processing time

    private final String mNetworkTablesKey;
    private final Pose2d mCameraOffset;
    private VisionUpdate mLatestVisionUpdate = null;

    private VisionUpdateManager(String coprocessorID, Pose2d cameraOffset)
    {
        mNetworkTablesKey = "/SmartDashboard/Vision/" + coprocessorID + "/solvePNP";
        mCameraOffset = cameraOffset;

        NetworkTableInstance.getDefault().addEntryListener(mNetworkTablesKey, (e) -> visionKeyChangedCallback(e),
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    private void visionKeyChangedCallback(EntryNotification entryNotification)
    {
        try
        {
            String rawVisionUpdate = entryNotification.value.getString();
            mLatestVisionUpdate = VisionUpdate.fromRawUpdate(rawVisionUpdate, mCameraOffset);
        }
        catch (Exception e)
        {
            Logger.exception(e);
            return;
        }

    }

    /**
     * @return either empty or contains the latest vision update
     */
    public Optional<VisionUpdate> getLatestVisionUpdate()
    {
        return Optional.ofNullable(mLatestVisionUpdate);
    }

    public static class VisionUpdate
    {

        public final double frameCapturedTime; // Time in seconds where the epoch the boot of the RoboRIO (getFPGATimestamp's epoch)
        public final Pose2d[] targetRobotRelativePositions; // The target's robot-relative position at frameCapturedTime (x and y in inches)

        private final Pose2d mCameraOffset;

        private VisionUpdate(double capturedTime, Pose2d cameraOffset,
                Pose2d... targetRelativePositions)
        {
            this.frameCapturedTime = capturedTime;
            this.targetRobotRelativePositions = targetRelativePositions;

            mCameraOffset = cameraOffset;
        }

        public static VisionUpdate fromRawUpdate(String vu, Pose2d cameraOffset)
        {
            String[] fields = vu.split(";"); // expect 2 or 3 fields
            int ntargets = fields.length - 1;
            if (ntargets <= 0)
                throw new RuntimeException("A vision update must have at least one target");

            // last field is timestamp
            double frameCapTime = Double.parseDouble(fields[ntargets]);

            Pose2d[] targets = new Pose2d[ntargets];
            for (int i = 0; i < ntargets; i++)
            {
                Double[] targetNumbers = Arrays.stream(fields[i].split(",")).map(Double::parseDouble).toArray(Double[]::new);
                if (targetNumbers.length < kRawUpdateNumDoubles)
                {
                    targets[i] = new Pose2d(targetNumbers[0], targetNumbers[1], Rotation2d.fromDegrees(targetNumbers[2]));
                }
                else
                {
                    throw new RuntimeException("A vision update must have at least " +
                            kRawUpdateNumDoubles + " doubles in the array. This one has " +
                            targetNumbers.length + ".");
                }
            }
            return new VisionUpdate(frameCapTime, cameraOffset, targets);
        }

        public Pose2d getFieldPosition(RobotStateMap stateMap)
        {
            // TODO: Look at NetworkTables to decide
            int index = 0;
            if (this.targetRobotRelativePositions.length <= index) throw new RuntimeException("targetRobotRelativePositions doesn't have index " + index + "!");

            return stateMap.getFieldToVehicle(this.frameCapturedTime).transformBy(mCameraOffset).transformBy(this.targetRobotRelativePositions[index]);
        }

        public Pose2d getCorrectedRobotPose(ScorableLandmark landmark, RobotStateMap stateMap, double timeToGetAt)
        {
            // TODO: Look at NetworkTables to decide
            int index = 0;
            if (this.targetRobotRelativePositions.length <= index) throw new RuntimeException("targetRobotRelativePositions doesn't have index " + index + "!");

            Pose2d robotPoseRelativeToLastVisionUpdate =
                    stateMap.get(this.frameCapturedTime).pose.transformBy(mCameraOffset).inverse().transformBy(stateMap.get(timeToGetAt).pose);
            return this.targetRobotRelativePositions[index].inverse().transformBy(landmark.fieldPose).transformBy(robotPoseRelativeToLastVisionUpdate);
        }

        public Pose2d getCorrectedRobotPoseForClosestTarget(RobotStateMap stateMap, double timeToGetAt)
        {
            double smallestTargetDistance = Double.POSITIVE_INFINITY;
            ScorableLandmark closestTargetPose = null;
            Pose2d robotPose = stateMap.getFieldToVehicle(timeToGetAt);

            for (ScorableLandmark l : Constants.ScorableLandmark.class.getEnumConstants())
            {
                double distance = robotPose.distance(l.fieldPose);
                if (distance < smallestTargetDistance)
                {
                    closestTargetPose = l;
                    smallestTargetDistance = distance;
                }
            }

            if (closestTargetPose == null)
                throw new RuntimeException("No vision targets are close! Is Constants.kVisionTargetLocations empty?");

            // TODO: Use NetworkTables to decide
            return getCorrectedRobotPose(closestTargetPose, stateMap, timeToGetAt);
        }

    }
}
