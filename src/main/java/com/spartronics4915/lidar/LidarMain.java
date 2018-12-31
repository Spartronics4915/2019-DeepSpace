package com.spartronics4915.lidar;

import com.spartronics4915.lidar.Looper;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.RobotStateMap;
import com.spartronics4915.lib.util.SegmentBuilder;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.geometry.Rotation2d;
import com.spartronics4915.lib.geometry.Twist2d;
import com.spartronics4915.lib.lidar.LidarProcessor;
import com.spartronics4915.lib.lidar.icp.Point;
import com.spartronics4915.lib.lidar.icp.SegmentReferenceModel;

public class LidarMain
{
    public static final RobotStateMap sRobotStateMap = new RobotStateMap();
    public static final SegmentReferenceModel sReferenceModel = new SegmentReferenceModel(
        new SegmentBuilder(new Point(53, 0)).
        verticalBy(30).
        horizontalBy(18).
        verticalBy(45).
        horizontalBy(-26).
        verticalBy(94).
        horizontalBy(-61.5).
        verticalBy(-49.5).
        horizontalBy(13.5).
        verticalBy(-39).
        horizontalBy(3.5).
        verticalBy(-35).
        horizontalBy(-3.5).
        verticalBy(-43).
        getSegments()
    );

    public static void main(String[] args)
    {
        Looper mLooper;
        LidarProcessor mLidarProcessor;
        Logger.setVerbosity("DEBUG");

        mLooper = new Looper();
        mLidarProcessor = new LidarProcessor(LidarProcessor.RunMode.kRunAsTest, sReferenceModel,
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
                Thread.sleep(1000);
                Logger.debug("Pose (in from origin): " + sRobotStateMap.getLatestFieldToVehicle().toString());
                // Logger.debug("Measured velocity (in/sec): " + sRobotStateMap.getLatestMeasuredVelocity().getValue().toString());
            }
            catch(Exception e)
            {
                Logger.exception(e);
            }
        }
    }
}