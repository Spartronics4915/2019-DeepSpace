package com.spartronics4915.frc2019.auto.actions;

import com.spartronics4915.lib.util.RobotStateMap;
import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.frc2019.subsystems.RobotStateEstimator;
import com.spartronics4915.lib.physics.DriveCharacterization;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.Timer;

import java.nio.file.Paths;
import java.util.List;

public class CollectCurvatureData implements Action
{

    private static final double kMaxPower = 0.4;
    private static final double kStartPower = 0.2;
    private static final double kStartTime = 0.25;
    private static final double kRampRate = 0.02;
    private static final Drive mDrive = Drive.getInstance();
    private static final RobotStateMap mRobotStateMap = RobotStateEstimator.getInstance().getEncoderRobotStateMap();

    private final ReflectingCSVWriter<DriveCharacterization.CurvatureDataPoint> mCSVWriter;
    private final List<DriveCharacterization.CurvatureDataPoint> mCurvatureData;
    private final boolean mReverse;

    private boolean isFinished = false;
    private double mStartTime = 0.0;

    /**
     * @param data     reference to the list where data points should be stored
     * @param highGear use high gear or low
     * @param reverse  if true drive in reverse, if false drive normally
     */

    public CollectCurvatureData(List<DriveCharacterization.CurvatureDataPoint> data, boolean reverse)
    {
        mCurvatureData = data;
        mReverse = reverse;
        mCSVWriter = new ReflectingCSVWriter<>(Paths.get(System.getProperty("user.home"), "CURVATURE_DATA.csv").toString(), DriveCharacterization.CurvatureDataPoint.class);

    }

    @Override
    public void start()
    {
        mDrive.setOpenLoop(new DriveSignal(kStartPower, kStartPower));
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update()
    {
        double t = Timer.getFPGATimestamp() - mStartTime;
        if (t < kStartTime)
        { //give the robot some time to accelerate before recording data
            return;
        }
        double rightPower = kStartPower + (t - kStartTime) * kRampRate;
        if (rightPower > kMaxPower)
        {
            isFinished = true;
            return;
        }
        final RobotStateMap.State state = mRobotStateMap.getLatestState();
        mDrive.setOpenLoop(new DriveSignal((mReverse ? -1.0 : 1.0) * kStartPower, (mReverse ? -1.0 : 1.0) * rightPower));
        mCurvatureData.add(new DriveCharacterization.CurvatureDataPoint(
                state.predictedVelocity.dx, state.predictedVelocity.dtheta,
                kStartPower, rightPower));
        mCSVWriter.add(mCurvatureData.get(mCurvatureData.size() - 1));
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
