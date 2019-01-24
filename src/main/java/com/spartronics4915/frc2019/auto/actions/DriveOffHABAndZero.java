package com.spartronics4915.frc2019.auto.actions;

import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.Util;

public class DriveOffHABAndZero implements Action {

    public enum HABLevel {
        PLATFORM(1), LEVEL_TWO(2);

        public final int numBumps;
        private HABLevel(int numBumps)
        {
            this.numBumps = numBumps;
        }
    }
    
    private static final double kBumpEpsilon = 5.0; // Degrees

    private final Drive mDrive;
    private final double mVelocity;
    private final double mFeedforward;
    private int mNumBumps = 0;

    public DriveOffHABAndZero(double velocity, double feedforward, HABLevel startingLevel)
    {
        mVelocity = velocity;
        mFeedforward = feedforward;
        mDrive = Drive.getInstance();
    }

    @Override
    public boolean isFinished()
    {
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
        mDrive.setVelocity(new DriveSignal(mVelocity, mVelocity), new DriveSignal(mFeedforward, mFeedforward));
    }

}