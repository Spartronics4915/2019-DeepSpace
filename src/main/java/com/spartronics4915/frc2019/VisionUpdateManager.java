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
        public final Pose2d targetRobotRelativePosition1; // The target's robot-relative position at frameCapturedTime (x and y in inches)
        public final Pose2d targetRobotRelativePosition2; // The target's robot-relative position at frameCapturedTime (x and y in inches)

        private VisionUpdate(double capturedTime, 
                            Pose2d targetRelativePosition)
        {
            this.frameCapturedTime = capturedTime;
            this.targetRobotRelativePosition1 = targetRelativePosition;
            this.targetRobotRelativePosition2 = null;
        }

        private VisionUpdate(double capturedTime, Pose2d pose1, Pose2d pose2)
        {
            this.frameCapturedTime = capturedTime;
            this.targetRobotRelativePosition1 = pose1;
            this.targetRobotRelativePosition2 = pose2;
        }

        public static VisionUpdate fromRawUpdate(String vu)
        {
            String[] fields = vu.split(";"); // expect 2 or 3 fields
            int ntargets = fields.length - 1; 
            if(ntargets <= 0)
                throw new RuntimeException("A vision update must have at least one target");
            // last field is timestamp
            double frameCapTime = Double.parseDouble(fields[ntargets]);
            Pose2d pose1=null, pose2=null;
            for(int i=0;i<ntargets;i++)
            {
                Double[] targetNumbers = Arrays.stream(fields[i].split(",")).map(Double::parseDouble).toArray(Double[]::new);
                if (targetNumbers.length < kRawUpdateNumDoubles)
                {
                    if(pose1 == null)
                        pose1 = new Pose2d(targetNumbers[0], targetNumbers[1], 
                                                Rotation2d.fromDegrees(targetNumbers[2]));
                    else
                    if(pose2 == null)
                        pose2 = new Pose2d(targetNumbers[0], targetNumbers[1], 
                                            Rotation2d.fromDegrees(targetNumbers[2]));
                    else
                        Logger.warning("VisionUpdate received too many samples");
                }
                else
                {
                    throw new RuntimeException("A vision update must have at least " + 
                            kRawUpdateNumDoubles + " doubles in the array. This one has " +
                            targetNumbers.length + ".");
                }
            }
           return new VisionUpdate(frameCapTime, pose1, pose2);
        }

        public Pose2d getFieldPosition(int index, RobotStateMap stateMap)
        {
            if(index == 0)
                return stateMap.getFieldToVehicle(this.frameCapturedTime).transformBy(targetRobotRelativePosition1);
            else
                return stateMap.getFieldToVehicle(this.frameCapturedTime).transformBy(targetRobotRelativePosition2);
        }

        public Pose2d getCorrectedRobotPose(int index, ScorableLandmark landmark, RobotStateMap stateMap, double timeToGetAt)
        {
            Pose2d robotPoseRelativeToLastVisionUpdate = stateMap.get(this.frameCapturedTime).pose.inverse().transformBy(stateMap.get(timeToGetAt).pose);
            if(index == 0)
                return this.targetRobotRelativePosition1.inverse().transformBy(landmark.fieldPose).transformBy(robotPoseRelativeToLastVisionUpdate);
            else
                return this.targetRobotRelativePosition2.inverse().transformBy(landmark.fieldPose).transformBy(robotPoseRelativeToLastVisionUpdate);
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

            return getCorrectedRobotPose(0, closestTargetPose, stateMap, timeToGetAt);
        }

    }
}
