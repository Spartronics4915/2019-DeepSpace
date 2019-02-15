package com.spartronics4915.frc2019.auto.actions;

import java.util.Arrays;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.lib.geometry.Rotation2d;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.Units;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DetermineWheelbaseDiameter implements Action
{
    private final Drive mDrive = Drive.getInstance();
    private double[] mAccumYawPitchRoll = new double [3];
    private double[] mInitialYawPitchRoll = new double [3];
    private double mEffectiveWheelbaseDiameter;

    @Override
    public boolean isFinished() {
        return (Math.abs(mAccumYawPitchRoll[2]) - Math.abs(mInitialYawPitchRoll[2]) >= 3600);
    }

    @Override
    public void update() {
        SmartDashboard.putString("accumHeading", Arrays.toString(mAccumYawPitchRoll));
        mDrive.getAccumGyro(mAccumYawPitchRoll);
    }

    @Override
    public void done() {
        mDrive.setOpenLoop(DriveSignal.BRAKE);

        mEffectiveWheelbaseDiameter = (Constants.kDriveWheelRadiusInches * 
            Math.abs(mDrive.getLeftEncoderDistance() - mDrive.getRightEncoderDistance()))
            / Math.abs(Units.degrees_to_radians(mAccumYawPitchRoll[2]));
        Logger.info("Effective Wheelbase Diameter is: " + mEffectiveWheelbaseDiameter);
        SmartDashboard.putString("Effective Wheelbase Diameter", "" + mEffectiveWheelbaseDiameter);
    }

    @Override
    public void start() {
        mDrive.setOpenLoop(new DriveSignal(0.25, -0.25));
        mDrive.setHeading(Rotation2d.identity());
        mDrive.getAccumGyro(mInitialYawPitchRoll);
    }
    
}