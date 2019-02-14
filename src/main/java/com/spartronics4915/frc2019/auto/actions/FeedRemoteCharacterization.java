package com.spartronics4915.frc2019.auto.actions;

import java.util.Arrays;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public class FeedRemoteCharacterization implements Action
{

    private final Drive mDrive;
    private final NetworkTableEntry mAutoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    private final NetworkTableEntry mTelemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");

    Number[] mOutputArray = new Number[10];

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
        
        // The script says to use ft/s, but we use radians because that works better for us

        // ft and rads/s
        double leftPosition = mDrive.getLeftEncoderDistance() / 12;
        double leftVelocity = mDrive.getLeftVelocityTicksPer100ms() / Constants.kDriveEncoderPPR * (2 * Math.PI) * 10;

        // ft and rad/s
        double rightPosition = mDrive.getRightEncoderDistance() / 12;
        double rightVelocity = mDrive.getRightVelocityTicksPer100ms() / Constants.kDriveEncoderPPR * (2 * Math.PI) * 10;

        // Yes, I know we're mixing ft and rad/s, but it's because the position value
        // isn't used except to display a sanity check on the remote script. Radians
        // are hard to read, so we use ft to make our sanity check more... sane.

        // volts
        double battery = RobotController.getBatteryVoltage();

        // volts
        double leftMotorVolts = mDrive.getLeftOutputVoltage();
        double rightMotorVolts = mDrive.getRightOutputVoltage();

        // percent output on [-1 1]
        double[] autospeed = mAutoSpeedEntry.getDoubleArray(new double[2]);

        mDrive.setOpenLoop(new DriveSignal(autospeed[0], autospeed[1]));

        // send telemetry data array back to NT
		mOutputArray[0] = now;
		mOutputArray[1] = battery;
        mOutputArray[2] = autospeed[0];
        mOutputArray[3] = autospeed[1];
		mOutputArray[4] = leftMotorVolts;
		mOutputArray[5] = rightMotorVolts;
		mOutputArray[6] = leftPosition;
		mOutputArray[7] = rightPosition;
		mOutputArray[8] = leftVelocity;
        mOutputArray[9] = rightVelocity;
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