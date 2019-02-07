package com.spartronics4915.frc2019.auto.actions;

import com.spartronics4915.frc2019.VisionUpdateManager;
import com.spartronics4915.frc2019.subsystems.RobotStateEstimator;
import com.spartronics4915.lib.util.RobotStateMap;
import edu.wpi.first.wpilibj.Timer;
import com.spartronics4915.lib.geometry.Pose2d;

public class ZeroOdometryFromVision implements Action
{
    public boolean Zeroed = false;
    public Pose2d Target;
    public double Start;
    public double VisionCaptureTime;
    
    @Override
    public boolean isFinished() {
        if (Zeroed)
        {
            return true;
        }
        return false;
    }

    @Override
    public void update() {
        VisionCaptureTime = VisionUpdateManager.reverseVisionManager.getLatestVisionUpdate().frameCapturedTime;
        if (VisionCaptureTime > Start) 
        {
            RobotStateEstimator.getInstance().getEncoderRobotStateMap().get(VisionCaptureTime);
            Zeroed = true;
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