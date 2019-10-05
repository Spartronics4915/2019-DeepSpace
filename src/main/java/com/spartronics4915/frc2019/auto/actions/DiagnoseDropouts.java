package com.spartronics4915.frc2019.auto.actions;

import java.nio.file.Paths;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.ReflectingCSVWriter;
import com.spartronics4915.lib.util.Units;

import edu.wpi.first.wpilibj.Timer;

public class DiagnoseDropouts implements Action
{

    private final DriveSignal kPercentDemand = new DriveSignal(0.5, 0.5);
    private final double kRunTime = 10.0; // Seconds

    private final Drive mDrive = Drive.getInstance();
    private final Timer mTimer = new Timer();
    private final ReflectingCSVWriter<VelocityTimeDataPoint> mCSVWriter;

    public DiagnoseDropouts()
    {
        mCSVWriter = new ReflectingCSVWriter<>(Paths.get(System.getProperty("user.home"), "DROPOUT_DATAs.csv").toString(), VelocityTimeDataPoint.class);
        mTimer.start();
        mTimer.reset();
    }
    
    @Override
    public boolean isFinished()
    {
        return mTimer.hasPeriodPassed(kRunTime);
    }

    @Override
    public void update()
    {
        mDrive.setOpenLoop(kPercentDemand);
        mCSVWriter.add(new VelocityTimeDataPoint(mDrive.getLeftVelocityTicksPer100ms()));
    }

    @Override
    public void done()
    {
        mCSVWriter.flush();
    }

    @Override
    public void start()
    {

    }

    public class VelocityTimeDataPoint
    {

        public final double velocity; // rps
        public final double time; // seconds since RoboRIO boot

        /**
         * @param velocity in ticks/100ms
         */
        public VelocityTimeDataPoint(double velocity)
        {
            this.velocity = Units.rads_per_sec_to_rpm(velocity / Constants.kDriveEncoderPPR * (2 * Math.PI) * 10);
            this.time = Timer.getFPGATimestamp();
        }
    }
}
