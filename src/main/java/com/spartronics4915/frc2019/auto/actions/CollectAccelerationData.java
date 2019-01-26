package com.spartronics4915.frc2019.auto.actions;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.auto.modes.CharacterizeDriveMode.SideToCharacterize;
import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.lib.physics.DriveCharacterization;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.ReflectingCSVWriter;
import com.spartronics4915.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;

import java.nio.file.Paths;
import java.util.List;

public class CollectAccelerationData implements Action
{

    private static final double kPower = 0.5;
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
     * @param data     reference to the list where data points should be stored
     * @param highGear use high gear or low
     * @param reverse  if true drive in reverse, if false drive normally
     * @param turn     if true turn, if false drive straight
     * @param side     the side to collect data for (motors will be run for all sides regardless of this setting)
     */
    public CollectAccelerationData(List<DriveCharacterization.AccelerationDataPoint> data, boolean reverse, boolean turn, SideToCharacterize side)
    {
        mAccelerationData = data;
        mReverse = reverse;
        mTurn = turn;
        mSide = side;
        mCSVWriter = new ReflectingCSVWriter<>(Paths.get(System.getProperty("user.home"), "ACCEL_DATA.csv").toString(), DriveCharacterization.AccelerationDataPoint.class);
    }

    @Override
    public void start()
    {
        mDrive.setOpenLoop(new DriveSignal(
            (mSide.shouldRunLeft() ? 1 : 0) * (mReverse ? -1.0 : 1.0) * kPower,
            (mSide.shouldRunRight() ? 1 : 0) * (mReverse ? -1.0 : 1.0) * (mTurn ? -1.0 : 1.0) * kPower)
        );
        mStartTime = Timer.getFPGATimestamp();
        mPrevTime = mStartTime;
        Logger.debug("Collecting acceleration data");
    }

    @Override
    public void update()
    {
        double currentVelocity =
                mSide.getVelocityTicksPer100ms(mDrive) / Constants.kDriveEncoderPPR * (2 * Math.PI) * 10;
        double currentTime = Timer.getFPGATimestamp();

        //don't calculate acceleration until we've populated prevTime and prevVelocity
        if (mPrevTime == mStartTime)
        {
            mPrevTime = currentTime;
            mPrevVelocity = currentVelocity;
            return;
        }

        double acceleration = (currentVelocity - mPrevVelocity) / (currentTime - mPrevTime);

        //ignore accelerations that are too small
        if (acceleration < Util.kEpsilon)
        {
            mPrevTime = currentTime;
            mPrevVelocity = currentVelocity;
            return;
        }

        mAccelerationData.add(new DriveCharacterization.AccelerationDataPoint(
                currentVelocity, //convert to radians per second
                kPower * 12.0, //convert to volts
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
        mDrive.setOpenLoop(DriveSignal.BRAKE);
        mCSVWriter.flush();
    }
}
