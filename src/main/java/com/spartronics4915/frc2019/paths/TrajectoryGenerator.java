package com.spartronics4915.frc2019.paths;

import com.spartronics4915.frc2019.planners.DriveMotionPlanner;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.geometry.Rotation2d;
import com.spartronics4915.lib.trajectory.TimedView;
import com.spartronics4915.lib.trajectory.Trajectory;
import com.spartronics4915.lib.trajectory.TrajectoryIterator;
import com.spartronics4915.lib.trajectory.TrajectoryUtil;
import com.spartronics4915.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.spartronics4915.lib.trajectory.timing.TimedState;
import com.spartronics4915.lib.trajectory.timing.TimingConstraint;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.frc2019.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryGenerator
{

    private static final double kMaxVelocity = 240.0; // inches/s
    private static final double kMaxAccel = 40.0; // inches/s
    private static final double kMaxCentripetalAccel = 30.0; // inches/s
    private static final double kMaxVoltage = 9.0; // volts

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance()
    {
        return mInstance;
    }

    private TrajectoryGenerator()
    {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories()
    {
        if (mTrajectorySet == null)
        {
            Logger.debug("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            Logger.debug("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet()
    {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints)
    {
        return mMotionPlanner.generateTrajectory(reversed, waypoints,
                Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel, // inches/s
            double max_accel, // inches/s^2
            double max_voltage)
    {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel, // inches/s
            double end_vel, // inches/s
            double max_vel, // inches/s
            double max_accel, // inches/s^2
            double max_voltage)
    {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x axis for LEFT)
    private static final Pose2d kRightCargoDepotIntakePose = new Pose2d(60.0, -90.0, Rotation2d.fromDegrees(160));

    public class TrajectorySet
    {

        public class MirrorableTrajectory
        {

            public MirrorableTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right)
            {
                this.right = right;
                this.left = TrajectoryUtil.mirrorTimed(right);
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left)
            {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }

        public final MirrorableTrajectory straightTestReverse;
        public final MirrorableTrajectory straightTestForward;
        public final MirrorableTrajectory curvedTest;
        public final MirrorableTrajectory driveToParallelHatchFromSide;
        public final MirrorableTrajectory driveToClosestCargoShipBayFromDepot;
        public final MirrorableTrajectory driveToClosestCargoShipBayFromSide;
        public final MirrorableTrajectory driveToDepot;
        public final MirrorableTrajectory driveToParallelHatchFromMiddle;
        public final MirrorableTrajectory driveToClosestCargoShipBayMiddle;
        // public final MirrorableTrajectory driveOffHabReverse;
        // public final MirrorableTrajectory driveOffHabForward;
        public final TrajectoryIterator<TimedState<Pose2dWithCurvature>> driveReverseToShootInBay;

        private TrajectorySet()
        {
            straightTestForward = new MirrorableTrajectory(getStraightTestForward());
            straightTestReverse = new MirrorableTrajectory(getStraightTestReverse());
            curvedTest = new MirrorableTrajectory(getCurvedTest());
            driveToParallelHatchFromSide = new MirrorableTrajectory(getDriveToDriverStationParallelHatch());
            driveToClosestCargoShipBayFromDepot = new MirrorableTrajectory(getDriveToClosestCargoShipBayFromDepot());
            driveToClosestCargoShipBayMiddle = new MirrorableTrajectory(getDriveToClosestCargoShipBayFromMiddle());
            driveToDepot = new MirrorableTrajectory(getDriveToDepot());
            driveToParallelHatchFromMiddle = new MirrorableTrajectory(getDriveToParallelCargoBayFromMiddle());
            // driveOffHabForward = new MirrorableTrajectory(getDriveOffHabForward());
            // driveOffHabReverse = new MirrorableTrajectory(getDriveOffHabReverse());
            driveToClosestCargoShipBayFromSide = new MirrorableTrajectory(getDriveToClosestCargoShipBayFromSide());
            driveReverseToShootInBay = new TrajectoryIterator<>(new TimedView<>(getDriveBackToShootBay()));
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStraightTestForward()
        {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(0d, 0d, Rotation2d.identity()));
            waypoints.add(new Pose2d(120d, 0d, Rotation2d.identity()));
            return generateTrajectory(false, waypoints);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStraightTestReverse()
        {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(0d, 0d, Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(120d, 0d, Rotation2d.fromDegrees(180)));
            return generateTrajectory(true, waypoints);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getCurvedTest()
        {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(0d, 0d, Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(78d, 78d, Rotation2d.fromDegrees(90)));
            return generateTrajectory(true, waypoints);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getDriveToDriverStationParallelHatch()
        {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.kRightRobotLocationOnPlatform);
            waypoints.add(Constants.kRightRobotLocationOffPlatform);
            waypoints.add(Constants.ScorableLandmark.RIGHT_DRIVERSTATION_PARALLEL_CARGO_BAY.robotLengthCorrectedPose);

            return generateTrajectory(true, waypoints);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getDriveToClosestCargoShipBayFromDepot()
        {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightCargoDepotIntakePose);
            waypoints.add(Constants.ScorableLandmark.RIGHT_CLOSE_CARGO_BAY.robotLengthCorrectedPose);
            return generateTrajectory(true, waypoints);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getDriveToClosestCargoShipBayFromMiddle()
        {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.kMiddleRobotLocationOnPlatformForward);
            waypoints.add(Constants.kMiddleRobotLocationOffPlatformForward);
            waypoints.add(new Pose2d(195, -95, Rotation2d.fromDegrees(-23))); // Middle of field
            waypoints.add(Constants.ScorableLandmark.RIGHT_CLOSE_CARGO_BAY.robotLengthCorrectedPose);
            return generateTrajectory(true, waypoints);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getDriveToClosestCargoShipBayFromSide()
        {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.kRightRobotLocationOnPlatform);
            waypoints.add(Constants.kRightRobotLocationOffPlatform);
            waypoints.add(new Pose2d(190, -90, Rotation2d.fromDegrees(140)));
            waypoints.add(Constants.ScorableLandmark.RIGHT_CLOSE_CARGO_BAY.robotLengthCorrectedPose);
            return generateTrajectory(true, waypoints);
        }

        // Rotation is 0 because we haven't reset the pose yet
        // private Trajectory<TimedState<Pose2dWithCurvature>> getDriveOffHabReverse()
        // {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(0, 0, Rotation2d.identity()));
        //     waypoints.add(new Pose2d(-kDriveOffHabDistance, 0, Rotation2d.identity()));
        //     return generateTrajectory(true, waypoints);
        // }

        // Rotation is 0 because we haven't reset the pose yet
        // private Trajectory<TimedState<Pose2dWithCurvature>> getDriveOffHabForward()
        // {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(0, 0, Rotation2d.identity()));
        //     waypoints.add(new Pose2d(kDriveOffHabDistance, 0, Rotation2d.identity()));
        //     return generateTrajectory(false, waypoints);
        // }

        private Trajectory<TimedState<Pose2dWithCurvature>> getDriveBackToShootBay()
        {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(Constants.kShootIntoBayBackupDistance, 0, Rotation2d.fromDegrees(0)));
            return generateTrajectory(false, waypoints);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getDriveToDepot()
        {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.ScorableLandmark.RIGHT_DRIVERSTATION_PARALLEL_CARGO_BAY.robotLengthCorrectedPose);
            waypoints.add(kRightCargoDepotIntakePose);
            return generateTrajectory(false, waypoints);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getDriveToParallelCargoBayFromMiddle()
        {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.kMiddleRobotLocationOnPlatformReverse);
            waypoints.add(Constants.kMiddleRobotLocationOffPlatformReverse);
            waypoints.add(Constants.ScorableLandmark.RIGHT_DRIVERSTATION_PARALLEL_CARGO_BAY.robotLengthCorrectedPose);
            return generateTrajectory(true, waypoints);
        }
    }
}
