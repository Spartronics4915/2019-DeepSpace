package com.spartronics4915.frc2019.auto.actions;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.frc2019.subsystems.RobotStateEstimator;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.util.RobotStateMap;

import edu.wpi.first.wpilibj.Timer;

public class ZeroOdometryOffHAB implements Action
{

    private RobotStateMap mStateMap;
    private boolean mStartOnLeft;

    public ZeroOdometryOffHAB(boolean left)
    {
        mStartOnLeft = left;
        mStateMap = RobotStateEstimator.getInstance().getEncoderRobotStateMap();
    }

    @Override
    public boolean isFinished()
    {
        return true;
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
        Pose2d pose;
        if (mStartOnLeft)
        {
            pose = Constants.kRightRobotLocationOffPlatform.mirror();
            mStateMap.reset(Timer.getFPGATimestamp(),
                    pose);
        }
        else
        {
            pose = Constants.kRightRobotLocationOffPlatform;
            mStateMap.reset(Timer.getFPGATimestamp(),
                    pose);
        }
        Drive.getInstance().setHeading(pose.getRotation());
    }

}
