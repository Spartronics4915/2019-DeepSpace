package com.spartronics4915.frc2019.auto.actions;

import com.spartronics4915.frc2019.VisionUpdateManager;
import com.spartronics4915.frc2019.Constants.ScorableLandmark;
import com.spartronics4915.frc2019.subsystems.RobotStateEstimator;
import com.spartronics4915.lib.util.RobotStateMap;
import edu.wpi.first.wpilibj.Timer;

public class ZeroOdometryFromVision implements Action
{

    private final ScorableLandmark kTargetLandmark;

    private boolean mZeroed = false;
    private double mStartTime;
    private double mVisionCaptureTime;
    private RobotStateEstimator mStateEstimator;

    public ZeroOdometryFromVision(ScorableLandmark landmark)
    {
        kTargetLandmark = landmark;
        mStateEstimator = RobotStateEstimator.getInstance();
    }

    @Override
    public boolean isFinished()
    {
        return mZeroed;
    }

    @Override
    public void update()
    {
        VisionUpdateManager.reversePNPVisionManager.getLatestVisionUpdate().ifPresent(visionUpdate ->
        {
            mVisionCaptureTime = visionUpdate.frameCapturedTime;
            if (mVisionCaptureTime > mStartTime)
            {
                mStateEstimator.resetRobotStateMaps(
                        visionUpdate.getCorrectedRobotPose(kTargetLandmark, mStateEstimator.getEncoderRobotStateMap(), Timer.getFPGATimestamp()));

                mZeroed = true;
            }
        });
    }

    @Override
    public void done()
    {

    }

    @Override
    public void start()
    {
        mStartTime = Timer.getFPGATimestamp();
    }

}
