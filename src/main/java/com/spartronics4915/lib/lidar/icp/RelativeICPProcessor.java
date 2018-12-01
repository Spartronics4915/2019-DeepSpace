package com.spartronics4915.lib.lidar.icp;

import com.spartronics4915.lib.geometry.Pose2d;

public class RelativeICPProcessor
{

    private final ICP mICP;

    private IReferenceModel mLastReferenceModel;
    private Transform mLastTransform;

    /**
     * Instantiate a RelativeICPProcessor and have it make its own ICP object. You
     * must specify the convergence timeout for the ICP object that will be made.
     * 
     * @param icpTimeoutMs Convergence timeout for ICP
     */
    public RelativeICPProcessor(long icpTimeoutMs)
    {
        mICP = new ICP(icpTimeoutMs);
    }

    /**
     * Instantiate a RelativeICPProcessor with a caller-supplied ICP object (e.g.
     * you alread have one made, and don't want to instantiate a whole new one)
     * 
     * @param icp Premade ICP object
     */
    public RelativeICPProcessor(ICP icp)
    {
        mICP = icp;
    }

    /**
     * Applies ICP point registration, using the last provided point cloud as a
     * reference field and the last robot position as a guess. Assumes (and will
     * return) that the robot is at 0, 0 on the first call to this method.
     * 
     * @param pointCloud
     * @param vehicleToLidar Pose2d that represents the vehicle to lidar transform
     * @return The robot's transform
     */
    public Transform doRelativeICP(Iterable<Point> pointCloud, Pose2d vehicleToLidar)
    {
        return doRelativeICP(pointCloud, new Transform(mLastTransform.toPose2d().transformBy(vehicleToLidar)));
    }

    /**
     * Applies ICP point registration, using the last provided point cloud as a
     * reference field, and a caller-provided guess (e.g. from odometry) instead of
     * using the last computed transform as a guess. Assumes (and will return) that
     * the robot is at 0, 0 on the first call.
     * 
     * @param pointCloud
     * @param guess      Caller-provided guess. Could be from odometry, but must be
     *                   in lidar coordinates.
     * @return The robot's transform
     */
    public Transform doRelativeICP(Iterable<Point> pointCloud, Transform guess)
    {
        Transform t = mLastReferenceModel == null ? new Transform() : mICP.doICP(pointCloud, guess, mLastReferenceModel);
        Transform newTransform = new Transform(mLastTransform.toPose2d().transformBy(t.toPose2d()));
        mLastTransform = t;
        mLastReferenceModel = new PointCloudReferenceModel(pointCloud); // Store this just for completeness
        return newTransform;
    }
}
