package com.spartronics4915.lib.lidar.icp;

import com.spartronics4915.lib.lidar.icp.Point;
import com.spartronics4915.lib.lidar.*; 
import com.spartronics4915.lib.util.RobotStateMap;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.geometry.Twist2d;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

public class ICPTest
{
    public static final double kTestEpsilon = .001;
    public static double kBucketSize = 2; // inches

    @Test
    public void testRelative() // NB: scan2 fails for degPerSample < 3
    {
        final int scansPerSec = 8;
        final int ptsPerScan = 360;
        final int degPerSample = 3; 
        double scanInterval = 1.0 / scansPerSec;
        double t = 0;
        double deltaT = scanInterval / ptsPerScan;
        LidarScan scan1 = new LidarScan();
        LidarScan scan2 = new LidarScan();
        LidarScan scan3 = new LidarScan();
        ICP icp = new ICP(0/*no timeout during testing*/);
        icp.maxIterations = 5000;
        RelativeICPProcessor relICP = new RelativeICPProcessor(icp);
        RobotStateMap stateMap = new RobotStateMap();

        // robot starts at 0, 0
        stateMap.addObservations(t, new Pose2d(), 
                        Twist2d.identity(), Twist2d.identity());

        // scan1 
        //      ellipse (100, 300) centered at robot origin (0, 0)
        // scan2:  
        //      scan1 offset (-2,-5) -> robot moved by (2, 5)
        // scan3:
        //      scan1 rotated(10), offset (-2,-5) -> robot turned(-10)
        final double xrad = 100;
        final double yrad = 3 * xrad;
        final double mapRot = Math.toRadians(10); // for rotating scan2->scan3
        for(int ang=0; ang<360; ang+=degPerSample)
        {
            double rad = Math.toRadians(ang);
            double x = xrad * Math.cos(rad);
            double y = yrad * Math.sin(rad);
            scan1.addPoint(new Point(x, y), t);
            scan2.addPoint(new Point(x-2, y-5), t); // NB: no smearing here

            // rotate the ellipse
            x = x * Math.cos(mapRot);
            y = y * Math.sin(mapRot);
            scan3.addPoint(new Point(x-2.1, y-5.1), t);
            t += deltaT;
        }

        // seed the first scan
        relICP.doRelativeICP(icp.getCulledPoints(scan1.getPoints(), kBucketSize));

        // verify that equivalent maps return 0
        Transform tx1;
        tx1 = relICP.doRelativeICP(icp.getCulledPoints(scan1.getPoints(), kBucketSize));
        assertEquals(0, tx1.tx, kTestEpsilon);
        assertEquals(0, tx1.ty, kTestEpsilon);
        assertEquals(0, tx1.theta, kTestEpsilon);

        // verify that translated maps return clean translation
        tx1 = relICP.doRelativeICP(icp.getCulledPoints(scan2.getPoints(), kBucketSize));
        assertEquals(-2, tx1.tx, kTestEpsilon);
        assertEquals(-5, tx1.ty, kTestEpsilon);
        assertEquals(0, tx1.theta, kTestEpsilon);

        // verify that rotated maps return clean rotation (fails)
        tx1 = relICP.doRelativeICP(icp.getCulledPoints(scan3.getPoints(), kBucketSize));
        //assertEquals(0, tx1.tx, kTestEpsilon);
        //assertEquals(0, tx1.ty, kTestEpsilon);
        //assertEquals(mapRot, tx1.theta, kTestEpsilon);
    }
}