package com.spartronics4915.frc2019.auto.actions;

import com.spartronics4915.lib.util.RobotStateMap;
import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.frc2019.subsystems.RobotStateEstimator;
import com.spartronics4915.lib.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.trajectory.TimedView;
import com.spartronics4915.lib.trajectory.Trajectory;
import com.spartronics4915.lib.trajectory.TrajectoryIterator;
import com.spartronics4915.lib.trajectory.timing.TimedState;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.Timer;

public class DriveTrajectory implements Action
{

    private static final Drive mDrive = Drive.getInstance();
    private static final RobotStateMap mRobotStateMap = RobotStateEstimator.getInstance().getEncoderRobotStateMap();

    private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTrajectory;
    private final boolean mResetPose;

    public DriveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory)
    {
        this(trajectory, false);
    }

    public DriveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, boolean resetPose)
    {
        mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        mResetPose = resetPose;
    }

    @Override
    public boolean isFinished()
    {
        if (mDrive.isDoneWithTrajectory())
        {
            Logger.debug("Trajectory finished");
            return true;
        }
        return false;
    }

    @Override
    public void update()
    {
    }

    @Override
    public void done()
    {
    }

    @Override
    public void start()
    {
        Logger.debug("Starting trajectory! (length=" + mTrajectory.getRemainingProgress() + ")");
        if (mResetPose)
        {
            mRobotStateMap.reset(Timer.getFPGATimestamp(), 
                mTrajectory.getState().state().getPose());
        }
        mDrive.setTrajectory(mTrajectory);
    }
}
