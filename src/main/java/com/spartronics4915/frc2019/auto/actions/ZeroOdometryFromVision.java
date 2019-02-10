package com.spartronics4915.frc2019.auto.actions;

import com.spartronics4915.frc2019.VisionUpdateManager;
import com.spartronics4915.frc2019.subsystems.RobotStateEstimator;
import com.spartronics4915.lib.util.RobotStateMap;
import edu.wpi.first.wpilibj.Timer;
import com.spartronics4915.lib.geometry.Pose2d;

public class ZeroOdometryFromVision implements Action
{

    private final Pose2d kTargetFieldPos;

    private boolean mZeroed = false;
    private double mStartTime;
    private double mVisionCaptureTime;
    private RobotStateMap mStateMap;

    public ZeroOdometryFromVision(Pose2d targetFieldPos)
    {
        kTargetFieldPos = targetFieldPos;
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
        VisionUpdateManager.VisionUpdate visionUpdate = VisionUpdateManager.reverseVisionManager.getLatestVisionUpdate();
        if (visionUpdate != null)
        {
            mVisionCaptureTime = visionUpdate.frameCapturedTime;
            if (mVisionCaptureTime > mStartTime)
            {
                double now = Timer.getFPGATimestamp();
                mStateMap.reset(now, visionUpdate.getCorrectedRobotPose(kTargetFieldPos, mStateMap, now));

                mZeroed = true;
            }
        }
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
