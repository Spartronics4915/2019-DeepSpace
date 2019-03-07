package com.spartronics4915.frc2019.auto.actions;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.frc2019.subsystems.RobotStateEstimator;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.util.RobotStateMap;

import edu.wpi.first.wpilibj.Timer;

public class ZeroOdometryOffHAB implements Action
{
    private StartPosition mStartPosition;
    private RobotStateMap mStateMap;
    public enum StartPosition
    {
        LEFT_PLATFORM,
        MIDDLE_PLATFORM,
        RIGHT_PLATFORM;
    };

    public ZeroOdometryOffHAB(StartPosition startPos)
    {
        mStartPosition = startPos;
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
        switch (mStartPosition)
        {
            case RIGHT_PLATFORM:
                pose = Constants.kRightRobotLocationOffPlatform.mirror();
                break;
            case LEFT_PLATFORM:
                pose = Constants.kRightRobotLocationOffPlatform;
                break;
            case MIDDLE_PLATFORM:
                pose = Constants.kRobotMiddleLocationOffPlatform;
                break;
            default:
                pose = new Pose2d();
                throw new RuntimeException("Invalid starting position " + mStartPosition);
        }

        mStateMap.reset(Timer.getFPGATimestamp(),
                     pose);
        Drive.getInstance().setHeading(pose.getRotation());
    }

}
