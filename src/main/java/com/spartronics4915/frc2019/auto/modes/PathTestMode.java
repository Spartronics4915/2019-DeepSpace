package com.spartronics4915.frc2019.auto.modes;

import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.AutoModeEndedException;
import com.spartronics4915.frc2019.auto.actions.DriveTrajectory;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator;
import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.lib.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.trajectory.Trajectory;
import com.spartronics4915.lib.trajectory.timing.TimedState;

public class PathTestMode extends AutoModeBase
{
        private boolean mCurved;

        public PathTestMode(boolean curved)
        {
                mCurved = curved;
        }

        @Override
        protected void routine() throws AutoModeEndedException
        {
                Drive.getInstance().startLogging();

                Trajectory<TimedState<Pose2dWithCurvature>> t = mCurved ?
                        TrajectoryGenerator.getInstance().getTrajectorySet().curvedTest.get(true) :
                        TrajectoryGenerator.getInstance().getTrajectorySet().straightTest.get(true);
                runAction(new DriveTrajectory(t, true));

                Drive.getInstance().stopLogging();

                /*
                 * runAction(new
                 * DriveTrajectory(TrajectoryGenerator.getInstance().generateTrajectory(
                 * false,
                 * Arrays.asList(Pose2d.identity(), Pose2d.fromTranslation(new
                 * Translation2d(48.0, 0.0)),
                 * new Pose2d(new Translation2d(96.0, -48.0), Rotation2d.fromDegrees(-90.0)),
                 * new Pose2d(new Translation2d(96.0, -96.0), Rotation2d.fromDegrees(-90.0))),
                 * Arrays.asList(new CentripetalAccelerationConstraint(80.0)),
                 * 120.0, 120.0, 10.0),true));
                 */

                /*
                 * runAction(new
                 * DriveTrajectory(TrajectoryGenerator.getInstance().generateTrajectory(
                 * false,
                 * Arrays.asList(
                 * new Pose2d(new Translation2d(96.0, -96.0), Rotation2d.fromDegrees(90.0)),
                 * new Pose2d(new Translation2d(96.0, -48.0), Rotation2d.fromDegrees(90.0)),
                 * Pose2d.fromRotation(Rotation2d.fromDegrees(180.0))),
                 * Arrays.asList(new CentripetalAccelerationConstraint(120.0)),
                 * 120.0, 120.0, 10.0)));
                 */
        }
}
