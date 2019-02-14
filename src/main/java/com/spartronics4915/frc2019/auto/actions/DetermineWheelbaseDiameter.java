package com.spartronics4915.frc2019.auto.actions;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.Units;

public class DetermineWheelbaseDiameter implements Action
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

    @Override
    public void done() {
        mDrive.setOpenLoop(DriveSignal.BRAKE);
        mEffectiveWheelbaseDiameter = (Constants.kDriveWheelRadiusInches * 
            Math.abs(mDrive.getLeftEncoderDistance() - mDrive.getRightEncoderDistance()))
            / Units.degrees_to_radians(mAccumYawPitchRoll[2]);
        mDrive.logInfo("Effective Wheelbase Diameter is: " + mEffectiveWheelbaseDiameter);

    }

    @Override
    public void start() {
        mDrive.setOpenLoop(new DriveSignal(0.25, -0.25));

    }
    
}