package com.spartronics4915.frc2019;

import java.util.Optional;
import java.util.function.BiFunction;

import com.spartronics4915.frc2019.Constants.ScorableLandmark;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.geometry.Rotation2d;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.RobotStateMap;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class VisionUpdateManager<U extends IVisionUpdate>
{
    private static final Pose2d kReverseCameraOffset = new Pose2d(-7.5, 0, Rotation2d.fromDegrees(180));
    private static final RuntimeException kEmptyUpdateException = new RuntimeException("VisionUpdate targets is null or doesn't have specified index!");

    public static VisionUpdateManager<PNPUpdate> reversePNPVisionManager = new VisionUpdateManager<>(PNPUpdate::new, "Reverse", "solvePNP", kReverseCameraOffset);
    // public static VisionUpdateManager<HeadingUpdate> reverseHeadingVisionManager = new VisionUpdateManager<>(HeadingUpdate::new, "Reverse", "heading", kReverseCameraOffset);

    private final String mNetworkTablesKey;
    private final Pose2d mCameraOffset;
    private final BiFunction<double[], Pose2d, U> mUpdateConstructor;

    private U mLatestVisionUpdate = null;

    private VisionUpdateManager(BiFunction<double[], Pose2d, U> updateConstructor, String coprocessorID, String updateTypeName, Pose2d cameraOffset)
    {
        mNetworkTablesKey = "/SmartDashboard/Vision/" + coprocessorID + "/solvePNP";
        mCameraOffset = cameraOffset;
        mUpdateConstructor = updateConstructor;

        NetworkTableInstance.getDefault().addEntryListener(mNetworkTablesKey, (e) -> visionKeyChangedCallback(e),
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    private void visionKeyChangedCallback(EntryNotification entryNotification)
    {
        try
        {
            double[] rawVisionUpdate = entryNotification.value.getDoubleArray();
            mLatestVisionUpdate = mUpdateConstructor.apply(rawVisionUpdate, mCameraOffset);
        }
        catch (Exception e)
        {
            Logger.exception(e);
        }
    }

    /**
     * @return either empty or contains the latest vision update
     */
    public Optional<U> getLatestVisionUpdate()
    {
        return (mLatestVisionUpdate == null || mLatestVisionUpdate.isEmpty()) ? Optional.empty() : Optional.ofNullable(mLatestVisionUpdate);
    }

    public static class PNPUpdate implements IVisionUpdate
    {
        public  final double frameCapturedTime;

        private final Pose2d[] mTargets;
        private final Pose2d mCameraOffset;

        public PNPUpdate(double[] values, Pose2d cameraOffset)
        {
            // a target is 3 numbers, we also expect one time, so
            // the valid lengths are 1, 4, 7  => 0, 1, 2 targets
            int len = values.length;
            int ntargets = (len == 7) ? 2 : (len == 4) ? 1 : 0;
            if (ntargets <= 0)
            {
                Logger.warning("A PNP vision update must have 1, 4, or 7 doubles");

                this.frameCapturedTime = 0;
                mTargets = null;
                mCameraOffset = null;
                return;
            }

            // last field is timestamp
            double frameCapTime = values[len - 1];
            Pose2d[] targets = new Pose2d[ntargets];
            for (int i = 0, j = 0; i < ntargets; i++, j += 3)
            {
                // We flip by 180 because I assumed that the vector pointed into the target, but they delivered
                // it so that it points out of the target... Oh well.
                targets[i] = new Pose2d(values[j + 0], values[j + 1], Rotation2d.fromDegrees(values[j + 2] + 180));
            }

            this.frameCapturedTime = frameCapTime;
            mTargets = targets;
            mCameraOffset = cameraOffset;
        }

        public Pose2d getFieldPosition(RobotStateMap stateMap)
        {
            // TODO: Look at NetworkTables to decide
            int index = 0;
            if (isEmpty() && mTargets.length <= index)
                throw kEmptyUpdateException;

            return stateMap.getFieldToVehicle(this.frameCapturedTime).transformBy(mCameraOffset)
                    .transformBy(mTargets[index]);
        }

        public Pose2d getCorrectedRobotPose(ScorableLandmark landmark, RobotStateMap stateMap, double timeToGetAt)
        {
            // TODO: Look at NetworkTables to decide
            int index = 0;
            if (isEmpty() && mTargets.length <= index)
                throw kEmptyUpdateException;

            Pose2d robotPoseRelativeToLastVisionUpdate =
                    stateMap.get(this.frameCapturedTime).pose.transformBy(mCameraOffset).inverse().transformBy(stateMap.get(timeToGetAt).pose);
            return mTargets[index].inverse().transformBy(landmark.fieldPose)
                    .transformBy(robotPoseRelativeToLastVisionUpdate);
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

        @Override
        public boolean isEmpty()
        {
            return mTargets == null && mTargets.length <= 0;
        }

    }

    public static class HeadingUpdate implements IVisionUpdate
    {
        private final TargetInfo[] mTargets;

        // cameraOffset translation is unused
        public HeadingUpdate(double[] values, Pose2d cameraOffset)
        {
            if (values.length <= 0 && values.length % 2 == 0)
            {
                Logger.warning("A heading vision update must have an even, positive number of doubles");

                mTargets = null;
                return;
            }

            // last field is timestamp
            TargetInfo[] targets = new TargetInfo[values.length];
            for (int i = 0, j = 0; i < values.length; i++, j += 2)
                targets[i] = new TargetInfo(Rotation2d.fromRadians(values[j]).rotateBy(cameraOffset.getRotation()), values[j + 1]);

            mTargets = targets;
        }

        public TargetInfo getTargetInfo()
        {
            // TODO: Look at NetworkTables
            int index = 0;
            if (isEmpty() && mTargets.length <= index)
                throw kEmptyUpdateException;

            return mTargets[index];
        }

        @Override
        public boolean isEmpty()
        {
            return mTargets != null && mTargets.length > 0;
        }
        
        public class TargetInfo
        {
            public final Rotation2d headingError;
            public final double heightError; // Pixels in the source camera

            public TargetInfo(Rotation2d heading, double heightError)
            {
                this.headingError = heading;
                this.heightError = heightError;
            }
        }

    }
    
}
