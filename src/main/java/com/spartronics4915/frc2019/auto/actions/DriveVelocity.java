package com.spartronics4915.frc2019.auto.actions;

import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.lib.util.DriveSignal;

public class DriveVelocity implements Action
{

    private static final Drive mDrive = Drive.getInstance();

    private final DriveSignal mVelocity, mFeedforward;

    public DriveVelocity(DriveSignal velocity, DriveSignal feedforward)
    {
        mVelocity = velocity;
        mFeedforward = feedforward;
    }
    
    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void update()
    {
        mDrive.setVelocity(mVelocity, mFeedforward);
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
