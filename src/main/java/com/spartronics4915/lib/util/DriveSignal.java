package com.spartronics4915.lib.util;

/**
 * A drivetrain command consisting of the left, right motor settings and whether
 * the brake mode is enabled.
 */
public class DriveSignal
{

    protected double mLeftMotor;
    protected double mRightMotor;
    protected boolean mBrakeMode;

    public DriveSignal(double left, double right)
    {
        this(left, right, false);
    }

    public DriveSignal(double left, double right, boolean brakeMode)
    {
        mLeftMotor = left;
        mRightMotor = right;
        mBrakeMode = brakeMode;
    }

    public static DriveSignal NEUTRAL = new DriveSignal(0, 0, false);
    public static DriveSignal BRAKE = new DriveSignal(0, 0, true);

    public double getLeft()
    {
        return mLeftMotor;
    }

    public double getRight()
    {
        return mRightMotor;
    }

    public boolean getBrakeMode()
    {
        return mBrakeMode;
    }

    public DriveSignal scale(double s)
    {
        return new DriveSignal(this.mLeftMotor*s, this.mRightMotor*s,
                            this.mBrakeMode);
    }

    @Override
    public String toString()
    {
        return "L: " + mLeftMotor + ", R: " + mRightMotor + (mBrakeMode ? ", BRAKE" : "");
    }
}
