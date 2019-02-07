package com.spartronics4915.frc2019.auto.actions;

import com.spartronics4915.frc2019.VisionUpdateManager;
import com.spartronics4915.frc2019.subsystems.RobotStateEstimator;
import com.spartronics4915.lib.util.RobotStateMap;
import edu.wpi.first.wpilibj.Timer;
import com.spartronics4915.lib.geometry.Pose2d;

public class ZeroOdometryFromVision implements Action
{
    public boolean mZeroed = false;
    public Pose2d Target;
    public double Start;
    public double mVisionCaptureTime;
    public RobotStateMap mStateMap;
    
    @Override
    public boolean isFinished() {
        if (mZeroed)
        {
            return true;
        }
        else 
        {
            return false;
        }
    }

    @Override
    public void update() {
        VisionUpdateManager.VisionUpdate visionUpdate = VisionUpdateManager.reverseVisionManager.getLatestVisionUpdate();
        if (visionUpdate != null)
        {
            mVisionCaptureTime =  visionUpdate.frameCapturedTime;
            if (mVisionCaptureTime > Start) 
            {
                mStateMap.reset(
                    Timer.getFPGATimestamp(),
                    visionUpdate.targetRelativePosition.transformBy(mStateMap.get(Timer.getFPGATimestamp()).pose.inverse()).transformBy(mStateMap.get(Timer.getFPGATimestamp()).pose.transformBy(mStateMap.get(mVisionCaptureTime).pose.inverse())));
            
                mZeroed = true;
            }
        }
    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        Start = Timer.getFPGATimestamp();
    }

}