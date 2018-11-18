package com.spartronics4915.lib.lidar.icp;

import java.util.ArrayList;

public class PointCloudReferenceModel implements IReferenceModel
{

    private Iterable<Point> mPointCloud;

    public PointCloudReferenceModel(Iterable<Point> pointCloud)
    {
        mPointCloud = pointCloud;
    }

    @Override
    public Point getClosestPoint(Point refPnt)
    {
        double minDist = Double.MAX_VALUE;
        Point minPnt = null;
        for (Point testPnt : mPointCloud)
        {
            if (refPnt.getDistanceSq(testPnt) < minDist)
                minPnt = testPnt;
        }
        return minPnt;
    }

    @Override
    public void transformBy(Transform t)
    {
        ArrayList<Point> transformedPoints = new ArrayList<Point>();
        for (Point p : mPointCloud)
        {
            transformedPoints.add(t.apply(p));
        }
        mPointCloud = transformedPoints;
    }

}
