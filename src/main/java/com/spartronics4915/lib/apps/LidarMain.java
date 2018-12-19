package com.spartronics4915.lib.apps;

import java.util.concurrent.TimeUnit;

import com.spartronics4915.lib.util.Looper;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.InterpolatingDouble;
import com.spartronics4915.lib.util.InterpolatingTreeMap;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.lidar.LidarProcessor;

public class LidarMain
{
    private static final int kPoseMapSize = 100;
    private static InterpolatingTreeMap<InterpolatingDouble, Pose2d> sPoses =
            new InterpolatingTreeMap<InterpolatingDouble, Pose2d>(kPoseMapSize);

    public static void main(String[] args)
    {
        Looper mLooper;
        LidarProcessor mLidarProcessor;
        Logger.setVerbosity("DEBUG");

        mLooper = new Looper();
        mLidarProcessor = new LidarProcessor(LidarProcessor.RunMode.kRunAsTest,
                                            null, null);
        mLooper.register(mLidarProcessor);
        boolean started = mLidarProcessor.isConnected();
        if(!started)
            return;

        Pose2d zeroPose = new Pose2d();
        double ts = System.currentTimeMillis() / 1000d;
        sPoses.put(new InterpolatingDouble(ts), zeroPose);
        mLooper.start();
        while (true)
        {
            try
            {
                // Lidar Processing occurs in LidarProcessor
                Thread.sleep(200);
            }
            catch(Exception e)
            {
                Logger.exception(e);
            }
        }
    }

    public static Pose2d getRobotPose(double timestamp)
    {
        return sPoses.get(new InterpolatingDouble(timestamp));
    }
}
