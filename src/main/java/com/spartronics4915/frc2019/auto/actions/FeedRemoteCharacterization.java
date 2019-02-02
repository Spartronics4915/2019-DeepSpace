package com.spartronics4915.frc2019.auto.actions;

import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.lib.util.DriveSignal;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public class FeedRemoteCharacterization implements Action
{

    private final Drive mDrive;
    private final NetworkTableEntry mAutoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    private final NetworkTableEntry mTelemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");

    private double mPriorAutospeed = 0;
    Number[] mOutputArray = new Number[9];


    /**
     * This feeds this python script: https://github.com/robotpy/robot-characterization
     */
    public FeedRemoteCharacterization()
    {
        mDrive = Drive.getInstance();
    }

    @Override
    public boolean isFinished()
    {
        // This must be manually stopped by the user via the "Disable button"
        return false;
    }

    @Override
    public void update()
    {
        double now = Timer.getFPGATimestamp();
        
        // ft and ft/s
        double leftPosition = mDrive.getLeftEncoderDistance();
        double leftVelocity = mDrive.getLeftLinearVelocity();

        // ft and ft/s
        double rightPosition = mDrive.getRightEncoderDistance();
        double rightVelocity = mDrive.getRightLinearVelocity();

        // volts
        double battery = RobotController.getBatteryVoltage();

        // volts
        double leftMotorVolts = mDrive.getLeftOutputVoltage();
        double rightMotorVolts = mDrive.getRightOutputVoltage();

        // percent output on [-1 1]
        double autospeed = mAutoSpeedEntry.getDouble(0);
        mPriorAutospeed = autospeed;

        mDrive.setOpenLoop(new DriveSignal(autospeed, autospeed));

        // send telemetry data array back to NT
		mOutputArray[0] = now;
		mOutputArray[1] = battery;
		mOutputArray[2] = autospeed;
		mOutputArray[3] = leftMotorVolts;
		mOutputArray[4] = rightMotorVolts;
		mOutputArray[5] = leftPosition;
		mOutputArray[6] = rightPosition;
		mOutputArray[7] = leftVelocity;
        mOutputArray[8] = rightVelocity;
        mTelemetryEntry.setNumberArray(mOutputArray);
    }

    @Override
    public void done()
    {
        mDrive.stop();
    }

    @Override
    public void start()
    {
        NetworkTableInstance.getDefault().setUpdateRate(0.010);
        mDrive.resetEncoders();
    }

}