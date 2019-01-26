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
    private final HABLevel mStartingLevel;
    private int mNumBumps = 0;
    private boolean mOnSlope = false;

    public DriveOffHABAndZero(double velocity, double feedforward, HABLevel startingLevel)
    {
        mVelocity = velocity;
        mFeedforward = feedforward;
        mDrive = Drive.getInstance();
        mStartingLevel = startingLevel;
    }

    @Override
    public boolean isFinished()
    {
        if (mNumBumps >= mStartingLevel.numBumps)
        {
            return true;
        }
        
        return false;
    }

    @Override
    public void update()
    {
        if (!mOnSlope && Math.abs(mDrive.getPitch().getDegrees()) > kBumpEpsilon)
        {
                mOnSlope = true;
        }
        
        else if (Math.abs(mDrive.getPitch().getDegrees()) < kBumpEpsilon && mOnSlope)
        {
            mOnSlope = false;
            mNumBumps++;
        }

    }

    @Override
    public void done()
    {
        mDrive.setOpenLoop(new DriveSignal(0, 0));
    }

    @Override
    public void start()
    {
        mDrive.setVelocity(new DriveSignal(mVelocity, mVelocity), new DriveSignal(mFeedforward, mFeedforward));
    }

}