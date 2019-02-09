package com.spartronics4915.frc2019.auto.actions;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.auto.modes.CharacterizeDriveMode.SideToCharacterize;
import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.lib.physics.DriveCharacterization;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.Timer;

import java.nio.file.Paths;
import java.util.List;

public class CollectVelocityData implements Action
{

    private static final double kMaxPower = 0.5;
    private static final double kRampRate = 0.02;
    private static final Drive mDrive = Drive.getInstance();

    private final ReflectingCSVWriter<DriveCharacterization.VelocityDataPoint> mCSVWriter;
    private final List<DriveCharacterization.VelocityDataPoint> mVelocityData;
    private final boolean mTurn;
    private final boolean mReverse;
    private final SideToCharacterize mSide;

    private boolean isFinished = false;
    private double mStartTime = 0.0;

    /**
     * This is a quasi-static test to determine Kv (the slope of of the voltage-speed curve).
     * This test collects data on robot (for convenience) so a regression can be run.
     * 
     * If you want to calculate the moment of inertia then you should run this test turning
     * in place. Otherwise you should go straight.
     * 
     * @param data     reference to the list where data points should be stored
     * @param reverse  if true drive in reverse, if false drive normally
     * @param turnInPlace     if true turn, if false drive straight
     * @param side     the side to collect data from/run
     */

    public CollectVelocityData(List<DriveCharacterization.VelocityDataPoint> data, boolean reverse, boolean turnInPlace, SideToCharacterize side)
    {
        mVelocityData = data;
        mReverse = reverse;
        mTurn = turnInPlace;
        mSide = side;
        mCSVWriter = new ReflectingCSVWriter<>(Paths.get(System.getProperty("user.home"), "VELOCITY_DATA.csv").toString(), DriveCharacterization.VelocityDataPoint.class);

    }

    @Override
    public void start()
    {
        mStartTime = Timer.getFPGATimestamp();
        Logger.debug("Collecting velocity data");
    }

    @Override
    public void update()
    {
        double percentPower = kRampRate * (Timer.getFPGATimestamp() - mStartTime);
        if (percentPower > kMaxPower)
        {
            isFinished = true;
            return;
        }
        mDrive.setOpenLoop(new DriveSignal(
            (mReverse ? -1.0 : 1.0) * percentPower,
            (mReverse ? -1.0 : 1.0) * (mTurn ? -1.0 : 1.0) * percentPower)
        );
        mVelocityData.add(new DriveCharacterization.VelocityDataPoint(
                mSide.getVelocityTicksPer100ms(mDrive) / Constants.kDriveEncoderPPR * (2 * Math.PI) * 10, //convert velocity to radians per second
                percentPower * 12.0 //convert to volts
        ));
        mCSVWriter.add(mVelocityData.get(mVelocityData.size() - 1));

    }

    @Override
    public boolean isFinished()
    {
        return isFinished;
    }

    @Override
    public void done()
    {
        mDrive.setOpenLoop(DriveSignal.BRAKE);
        mCSVWriter.flush();
    }
}
