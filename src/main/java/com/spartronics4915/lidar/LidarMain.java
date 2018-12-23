package com.spartronics4915.lidar;

import com.spartronics4915.lidar.Looper;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.RobotStateMap;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.lidar.LidarProcessor;

public class LidarMain
{
    public static final RobotStateMap sRobotStateMap = new RobotStateMap();

    public static void main(String[] args)
    {
        Looper mLooper;
        LidarProcessor mLidarProcessor;
        Logger.setVerbosity("DEBUG");

        mLooper = new Looper();
        mLidarProcessor = new LidarProcessor(LidarProcessor.RunMode.kRunAsTest, null,
                sRobotStateMap, sRobotStateMap, new Pose2d(), () -> System.currentTimeMillis() / 1000d);
        mLooper.register(mLidarProcessor);
        boolean started = mLidarProcessor.isConnected();
        if(!started)
        {
            Logger.error("Lidar is not connected");
            System.exit(1);
        }

        Runtime.getRuntime().addShutdownHook(new Thread() {
            @Override
            public void run() {
                synchronized (mLooper) {
                    mLooper.stop();
                    Runtime.getRuntime().halt(0); // System::exit will wait on this thread to finish, which is a deadlock
                }
            }
        });

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
}