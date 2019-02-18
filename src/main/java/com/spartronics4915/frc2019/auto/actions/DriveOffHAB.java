package com.spartronics4915.frc2019.auto.actions;

import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveOffHAB implements Action {

    public enum HABLevel {
        PLATFORM(1), LEVEL_TWO(2);

        public final int numBumps;
        private HABLevel(int numBumps)
        {
            this.numBumps = numBumps;
        }
    }
    
    private static final double kBumpMinDegrees = 5; // Degrees
    private static final double kOffBumpEpsilon = 3;

    private final Drive mDrive;
    private final double mVelocity;
    private final double mFeedforward;
    private final HABLevel mStartingLevel;
    private int mNumBumps = 0;
    private boolean mOnSlope = false;
    private double[] mYawPitchRollInitial = new double[3];
    private double[] mYawPitchRoll = new double[3];

    public DriveOffHAB(double velocity, double feedforward, HABLevel startingLevel)
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
        SmartDashboard.putNumber("Actions/DriveOffHAB/pitch", mYawPitchRoll[1] - mYawPitchRollInitial[1]);

        if (!mOnSlope && Math.abs(mYawPitchRoll[1] - mYawPitchRollInitial[1]) > kBumpMinDegrees)
        {
                Logger.debug("on bump");
                mOnSlope = true;
        }
        else if (Util.epsilonEquals(mYawPitchRoll[1] - mYawPitchRollInitial[1], 0, kOffBumpEpsilon) && mOnSlope)
        {
            Logger.debug("off bump");
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
        mYawPitchRollInitial = mDrive.getAccumGyro().clone();
        mYawPitchRoll = mDrive.getAccumGyro();
    }

}