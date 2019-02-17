package com.spartronics4915.frc2019.auto.actions;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.subsystems.RobotStateEstimator;
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

        if (mStartOnLeft)
        {
            mStateMap.reset(Timer.getFPGATimestamp(),
                    Constants.kRightRobotLocationOffPlatform.mirror());
        }
        else
        {
            mStateMap.reset(Timer.getFPGATimestamp(),
                    Constants.kRightRobotLocationOffPlatform);
        }
    }

}
