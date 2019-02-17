package com.spartronics4915.frc2019.auto.actions;

import com.spartronics4915.frc2019.VisionUpdateManager;
import com.spartronics4915.frc2019.Constants.FieldLandmark;
import com.spartronics4915.frc2019.subsystems.RobotStateEstimator;
import com.spartronics4915.lib.util.RobotStateMap;
import edu.wpi.first.wpilibj.Timer;

public class ZeroOdometryFromVision implements Action
{

    private final FieldLandmark kTargetLandmark;

    private boolean mZeroed = false;
    private double mStartTime;
    private double mVisionCaptureTime;
    private RobotStateMap mStateMap;

    public ZeroOdometryFromVision(FieldLandmark landmark)
    {
        kTargetLandmark = landmark;
        mStateMap = RobotStateEstimator.getInstance().getEncoderRobotStateMap();
    }

    @Override
    public boolean isFinished()
    {
        return mZeroed;
    }

    @Override
    public void update()
    {
        VisionUpdateManager.reverseVisionManager.getLatestVisionUpdate().ifPresent(visionUpdate ->
        {
            mVisionCaptureTime = visionUpdate.frameCapturedTime;
            if (mVisionCaptureTime > mStartTime)
            {
                double now = Timer.getFPGATimestamp();
                mStateMap.reset(now, visionUpdate.getCorrectedRobotPose(kTargetLandmark, mStateMap, now));

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
