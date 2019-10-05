package com.spartronics4915.frc2019.auto.actions;

import com.spartronics4915.frc2019.subsystems.RobotStateEstimator;
import edu.wpi.first.wpilibj.Timer;

public class WaitUntilCrossXBoundaryCommand implements Action
{

    private double mXBoundary = 0;

    public WaitUntilCrossXBoundaryCommand(double x)
    {
        mXBoundary = x;
    }

    @Override
    public boolean isFinished()
    {
        return RobotStateEstimator.getInstance().
            getEncoderRobotStateMap().getFieldToVehicle(Timer.getFPGATimestamp()).getTranslation().x() > mXBoundary;
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

    }
}
