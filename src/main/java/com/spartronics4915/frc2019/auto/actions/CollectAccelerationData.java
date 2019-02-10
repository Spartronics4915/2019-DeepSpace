package com.spartronics4915.frc2019.auto.actions;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.auto.modes.CharacterizeDriveMode.SideToCharacterize;
import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.lib.physics.DriveCharacterization;
import com.spartronics4915.lib.physics.DriveCharacterization.AccelerationDataPoint;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.ReflectingCSVWriter;
import com.spartronics4915.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.nio.file.Paths;
import java.util.List;

public class CollectAccelerationData implements Action
{

    private static final double kPower = 0.25;
    private static final double kTotalTime = 2.0; //how long to run the test for
    private static final Drive mDrive = Drive.getInstance();

    private final ReflectingCSVWriter<DriveCharacterization.AccelerationDataPoint> mCSVWriter;
    private final List<DriveCharacterization.AccelerationDataPoint> mAccelerationData;
    private final boolean mTurn;
    private final boolean mReverse;
    private final SideToCharacterize mSide;

    private double mStartTime = 0.0;
    private double mPrevVelocity = 0.0;
    private double mPrevTime = 0.0;

    /**
     * This test collects data about the behavior of the drivetrain in a "dynamic
     * test", where we set the demand to a constant value and sample acceleration.
     * The idea is that velocity is that steady-state velocity is negligle in this
     * case, so we are only measuring the effects of acceleration.
     * 
     * There are a couple of issues with this test (we should subtract the voltage
     * caused by velocity from this applied voltage), and one should examine if the
     * FeedRemoteCharacterization action is appropriate instead of this.
     * 
     * @param data        reference to the list where data points should be stored
     * @param reverse     if true drive in reverse, if false drive normally
     * @param turnInPlace if true turn, if false drive straight
     * @param side        the side to collect data for (motors will be run for all
     *                    sides regardless of this setting)
     */
    public CollectAccelerationData(List<DriveCharacterization.AccelerationDataPoint> data, boolean reverse, boolean turnInPlace,
            SideToCharacterize side)
    {
        mAccelerationData = data;
        mReverse = reverse;
        mTurn = turnInPlace;
        mSide = side;
        mCSVWriter = new ReflectingCSVWriter<>(Paths.get(System.getProperty("user.home"), "ACCEL_DATA.csv").toString(),
                DriveCharacterization.AccelerationDataPoint.class);
    }

    @Override
    public void start()
    {
        mDrive.setOpenLoop(new DriveSignal(
                (mReverse ? -1.0 : 1.0) * kPower,
                (mReverse ? -1.0 : 1.0) * (mTurn ? -1.0 : 1.0) * kPower));
        mStartTime = Timer.getFPGATimestamp();
        mPrevTime = mStartTime;
        Logger.debug("Collecting acceleration data");
    }

    @Override
    public void update()
    {
        // convert to radians/sec
        double currentVelocity =
                mSide.getVelocityTicksPer100ms(mDrive) / Constants.kDriveEncoderPPR * (2 * Math.PI) * 10;
        double currentTime = Timer.getFPGATimestamp();

        SmartDashboard.putNumber("CollectAccelerationData/currentTime", currentTime);

        // don't calculate acceleration until we've populated prevTime and prevVelocity
        if (mPrevTime == mStartTime)
        {
            mPrevTime = currentTime;
            mPrevVelocity = currentVelocity;
            return;
        }

        double acceleration = (currentVelocity - mPrevVelocity) / (currentTime - mPrevTime);

        // ignore accelerations that are effectively 0
        if (acceleration < Util.kEpsilon)
        {
            mPrevTime = currentTime;
            mPrevVelocity = currentVelocity;
            return;
        }

        mAccelerationData.add(new DriveCharacterization.AccelerationDataPoint(
                currentVelocity, // rads/sec
                kPower * mSide.getVoltage(mDrive), //convert to volts
                acceleration));

        mCSVWriter.add(mAccelerationData.get(mAccelerationData.size() - 1));

        mPrevTime = currentTime;
        mPrevVelocity = currentVelocity;
    }

    @Override
    public boolean isFinished()
    {
        return Timer.getFPGATimestamp() - mStartTime > kTotalTime;
    }

    @Override
    public void done()
    {
        // Remove anything before we hit our max acceleration (i.e. don't log the ramp-up period)
        AccelerationDataPoint maxDataPoint = mAccelerationData.stream()
                .max((AccelerationDataPoint dp0, AccelerationDataPoint dp1) -> Double.compare(Math.abs(dp0.acceleration), Math.abs(dp1.acceleration)))
                .orElseThrow();
        mAccelerationData.subList(0, mAccelerationData.indexOf(maxDataPoint)).clear();

        mDrive.setOpenLoop(DriveSignal.BRAKE);
        mCSVWriter.flush(); // CSV writer will have value around 0 removed, but the max accel trimming will not be applied
    }
}
