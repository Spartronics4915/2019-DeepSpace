package com.spartronics4915.frc2019.auto.actions;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.frc2019.subsystems.Drive.DriveControlState;
import com.spartronics4915.lib.util.DriveSignal;

public class DetermineWheelbaseRadius implements Action
{
    private final Drive mDrive = Drive.getInstance();
    private PigeonIMU mPigeon;
    private double[] mAccumYawPitchRoll = new double [3];
    private double mEffectiveWheelbaseDiameter;

    @Override
    public boolean isFinished() {
        return (mAccumYawPitchRoll[2] >= 3600);
    }

    @Override
    public void update() {
        mPigeon.getAccumGyro(mAccumYawPitchRoll);

    }
    
    private double degreesToRadians(double degrees)
    {
        return (degrees * Math.PI) / 180;

    }

    @Override
    public void done() {
        mDrive.setOpenLoop(DriveSignal.BRAKE);
        mEffectiveWheelbaseDiameter = (Constants.kDriveWheelRadiusInches * 
            Math.abs(mDrive.getLeftEncoderDistance() - mDrive.getRightEncoderDistance()))
            / degreesToRadians(mAccumYawPitchRoll[2]);
        mDrive.logInfo("Effective Wheelbase Diameter is: " + mEffectiveWheelbaseDiameter);

    }

    @Override
    public void start() {
        mDrive.setOpenLoop(new DriveSignal(0.5, -0.5));

    }
    
}