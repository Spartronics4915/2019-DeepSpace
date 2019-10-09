package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.Kinematics;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.util.RobotStateMap;
import com.spartronics4915.lib.util.Units;
import com.spartronics4915.lib.util.ILooper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.geometry.Rotation2d;
import com.spartronics4915.lib.geometry.Twist2d;
import com.spartronics4915.lib.hardware.sensors.T265Camera;
import com.spartronics4915.lib.hardware.sensors.T265Camera.CameraUpdate;
import com.spartronics4915.lib.lidar.LidarProcessor;

public class RobotStateEstimator extends Subsystem
{

    private static RobotStateEstimator sInstance = new RobotStateEstimator();

    public static RobotStateEstimator getInstance()
    {
        return sInstance;
    }

    /**
     * The LIDAR/encoder RobotStateMap objects represent two views
     * of the robot's current state (state is pose, velocity,
     * and distance driven).
     */
    private RobotStateMap mEncoderRobotState = new RobotStateMap();
    private RobotStateMap mLidarRobotState = new RobotStateMap();
    private RobotStateMap mCameraRobotState = new RobotStateMap();
    
    private Drive mDrive;
    private T265Camera mSLAMCamera;
    private LidarProcessor mLidarProcessor = null;

    private double mLeftPrevDist = 0.0;
    private double mRightPrevDist = 0.0;

    private static final Pose2d kZeroPose = Pose2d.identity();

    RobotStateEstimator()
    {
        mDrive = Drive.getInstance();
        try {
            mSLAMCamera = new T265Camera(new com.spartronics4915.lib.math.twodim.geometry.Pose2d(Units.inches_to_meters(-4.4375), Units.inches_to_meters(-12.0), com.spartronics4915.lib.math.twodim.geometry.Rotation2d.fromDegrees(-90)), 0.000001);
        } catch (Exception e) {
            logException("Couldn't instantiate T265 camera", e);
            mSLAMCamera = null;
        }
        /*
         * Warning: This starts the LIDAR server on robot boot.
         * This may need to be deferred until autonomousInit or
         * something (because of the rules).
         */
        /*
         * disable Lidar for 2019 robot
         * final Pose2d vehicleToLidar = new Pose2d(
         * Constants.kLidarXOffset, Constants.kLidarYOffset,
         * Rotation2d.fromDegrees(Constants.kLidarYawAngleDegrees)
         * );
         * mLidarProcessor = new LidarProcessor(LidarProcessor.RunMode.kRunInRobot,
         * Constants.kSegmentReferenceModel,
         * mEncoderRobotState,
         * mLidarRobotState,
         * vehicleToLidar,
         * () -> Timer.getFPGATimestamp());
         */

        logInitialized(true);
    }

    public RobotStateMap getBestRobotStateMap() {
        if (mSLAMCamera != null) {
            return mCameraRobotState;
        } else {
            return mEncoderRobotState;
        }
    }

    public void resetRobotStateMaps()
    {
        resetRobotStateMaps(kZeroPose);
    }

    public void resetRobotStateMaps(Pose2d pose)
    {
        double time = Timer.getFPGATimestamp();
        mEncoderRobotState.reset(time, pose);
        mLidarRobotState.reset(time, pose);
        mCameraRobotState.reset(time, pose);

        if (mSLAMCamera != null) {
            mSLAMCamera.setPose(new com.spartronics4915.lib.math.twodim.geometry.Pose2d());
        }
        mDrive.setHeading(pose.getRotation());
    }

    @Override
    public boolean checkSystem(String subsystem)
    {
        return false;
    }

    @Override
    public void outputTelemetry()
    {
        final RobotStateMap.State estate = getBestRobotStateMap().getLatestState();
        Pose2d epose = estate.pose;
        SmartDashboard.putString("RobotState/pose",
                epose.getTranslation().x() +
                        " " + epose.getTranslation().y() +
                        " " + epose.getRotation().getDegrees());
        Twist2d pVel = estate.predictedVelocity;
        SmartDashboard.putNumber("RobotState/velocity", pVel.dx);
    }

    @Override
    public void stop()
    {
        // No-op
    }

    @Override
    public void registerEnabledLoops(ILooper looper)
    {
        looper.register(new EnabledLoop());
        if (mLidarProcessor != null)
            looper.register(mLidarProcessor);
    }

    private class EnabledLoop implements ILoop
    {

        @Override
        public synchronized void onStart(double timestamp)
        {
            mLeftPrevDist = mDrive.getLeftEncoderDistance();
            mRightPrevDist = mDrive.getRightEncoderDistance();

            if (mSLAMCamera != null) {
                // Callback is called from a different thread... We avoid data races because RobotSteteMap is thread-safe
                mSLAMCamera.stop();
                mSLAMCamera.start((CameraUpdate update) ->
                {
                    var pose = new Pose2d(Units.meters_to_inches(update.pose.getTranslation().x()), Units.meters_to_inches(update.pose.getTranslation().y()), Rotation2d.fromDegrees(update.pose.getRotation().getDegrees()));
                    var velocity = new Twist2d(Units.meters_to_inches(update.velocity.dx), Units.meters_to_inches(update.velocity.dy), update.velocity.dtheta.getRadians());
                    mCameraRobotState.addObservations(Timer.getFPGATimestamp(), pose, velocity, new Twist2d());
                    SmartDashboard.putString("RobotState/cameraConfidence", update.confidence.toString());
                });
            }
        }

        @Override
        public synchronized void onLoop(double timestamp)
        {
            final RobotStateMap.State last = mEncoderRobotState.getLatestState();
            final Pose2d lastPose = last.pose;

            /* two ways to measure current velocity */
            /*
             * method 1, integrationVelocity
             * Look at the distance traveled since last measurement, consider
             * current gyro heading rather than our stored state
             * Divide by delta time to produce a velocity. Note that
             * 254's implementation doesn't include time computations explicitly.
             * In method 1, the implicit time is the time between samples which relates
             * to the looper time interval. Thus: leftDelta is measured in
             * inches/loopinterval. To the degree that the loop interval isn't a
             * constant the result will be noisy. OTH: we can interpret this
             * velocity as also a distance traveled since last loop.
             */
            final double leftDist = mDrive.getLeftEncoderDistance();
            final double rightDist = mDrive.getRightEncoderDistance();
            final double leftDelta = leftDist - mLeftPrevDist;
            final double rightDelta = rightDist - mRightPrevDist;
            final Rotation2d heading = mDrive.getHeading();
            mLeftPrevDist = leftDist;
            mRightPrevDist = rightDist;
            final Twist2d iVal = Kinematics.forwardKinematics(
                    lastPose.getRotation(),
                    leftDelta, rightDelta, heading);

            /*
             * method 2, 'predictedVelocity'
             * Directly sample the current wheel velocities. Here, linear velocities
             * are measured in inches/sec. Since the integration step below expects
             * velocity to be measured in inches/loopinterval, this version of velocity
             * can't be used directly. Moreover, the velocity we obtain from the wheel
             * encoders is integrated over a different time interval than one
             * loop-interval. It's not clear which estimation technique would deliver
             * a better result. For visualization purposes velocity2 (in inches/sec)
             * is in human-readable form. Also of note, this variant doesn't
             * include the gyro heading in its calculation.
             */
            final Twist2d pVal = Kinematics.forwardKinematics(
                    mDrive.getLeftLinearVelocity(),
                    mDrive.getRightLinearVelocity());

            /*
             * integrateForward: given a last state and a current velocity,
             * estimate a new state (P2 = P1 + dPdt * dt)
             */
            final Pose2d nextP = Kinematics.integrateForwardKinematics(last.pose, iVal);

            /* record the new state estimate */
            mEncoderRobotState.addObservations(timestamp, nextP, iVal, pVal);

            if (mSLAMCamera != null) {  
                // mSLAMCamera.sendOdometry(new com.spartronics4915.lib.math.twodim.geometry.Twist2d(Units.inches_to_meters(pVal.dx), 
                //        Units.inches_to_meters(pVal.dy), com.spartronics4915.lib.math.twodim.geometry.Rotation2d.fromRadians(pVal.dtheta)));
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            // no-op
        }
    }
}
