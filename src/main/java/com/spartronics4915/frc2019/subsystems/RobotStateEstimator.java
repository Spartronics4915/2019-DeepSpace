package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.Kinematics;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.util.RobotStateMap;
import com.spartronics4915.lib.util.ILooper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.geometry.Rotation2d;
import com.spartronics4915.lib.geometry.Twist2d;
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
    private Drive mDrive;
    private LidarProcessor mLidarProcessor;
    private double mLeftPrevDist = 0.0;
    private double mRightPrevDist = 0.0;

    private static final Pose2d kZeroPose = Pose2d.identity();

    RobotStateEstimator()
    {
        mDrive = Drive.getInstance();
        /*
            Warning: This starts the LIDAR server on robot boot.
            This may need to be deferred until autonomousInit or
            something (because of the rules).
        */
        final Pose2d vehicleToLidar = new Pose2d(
            Constants.kLidarXOffset, Constants.kLidarYOffset,
            Rotation2d.fromDegrees(Constants.kLidarYawAngleDegrees)
        );
        mLidarProcessor = new LidarProcessor(LidarProcessor.RunMode.kRunInRobot, 
                            Constants.kSegmentReferenceModel,
                            mEncoderRobotState,
                            mLidarRobotState,
                            vehicleToLidar,
                            () -> Timer.getFPGATimestamp());
    }

    public RobotStateMap getEncoderRobotStateMap()
    {
        return mEncoderRobotState;
    }

    public RobotStateMap getLidarRobotStateMap()
    {
        return mLidarRobotState;
    }

    public void resetRobotStateMaps(double timestamp)
    {
        mEncoderRobotState.reset(timestamp, kZeroPose);
        mLidarRobotState.reset(timestamp, kZeroPose);
    }

    @Override
    public boolean checkSystem(String subsystem)
    {
        return false;
    }

    @Override
    public void outputTelemetry()
    {
        Pose2d odometry = mEncoderRobotState.getLatestFieldToVehicle().getValue();
        SmartDashboard.putString("RobotState/pose",
                odometry.getTranslation().x() +
                        " " + odometry.getTranslation().y() +
                        " " + odometry.getRotation().getDegrees());
        Twist2d measuredVelocity = mEncoderRobotState.getLatestPredictedVelocity().getValue();
        SmartDashboard.putNumber("RobotState/velocity", measuredVelocity.dx);
        SmartDashboard.putNumber("RobotState/field_degrees", mEncoderRobotState.getLatestFieldToVehicle().getValue().getRotation().getDegrees());
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
        looper.register(mLidarProcessor);
    }

    private class EnabledLoop implements ILoop
    {

        @Override
        public synchronized void onStart(double timestamp)
        {
            mLeftPrevDist = mDrive.getLeftEncoderDistance();
            mRightPrevDist = mDrive.getRightEncoderDistance();
        }

        @Override
        public synchronized void onLoop(double timestamp)
        {
            /* two ways to measure current velocity, not sure which is better */

            /* method 1:  
            * Look at the distance traveled since last measurement, consider
            *   current gyro heading rather than our stored state
            * Divide by delta time to produce a velocity. Note that
            * 254's implementation doesn't include time computations explicitly.
            * In method 1, the implicit time is the time between samples which relates
            * to the looper time interval.  Thus: leftDelta is measured in
            * inches/loopinterval. To the degree that the loop interval isn't a
            * constant the result will be noisy.
            */
            final double leftDist = mDrive.getLeftEncoderDistance();
            final double rightDist = mDrive.getRightEncoderDistance();
            final double leftDelta = leftDist - mLeftPrevDist;
            final double rightDelta = rightDist - mRightPrevDist;
            final Rotation2d heading = mDrive.getHeading();
            mLeftPrevDist = leftDist;
            mRightPrevDist = rightDist;
            final Twist2d velocity1 = getVelocityFromDeltas(leftDelta, rightDelta, heading);

            /* method 2:
            *  Directly sample the current wheel velocities. Here, linear velocities
            *  are measured in inches/sec. Since the integration step below expects
            *  velocity to be measured in inches/loopinterval, this version of velocity
            *  can't be used directly. Moreover, the velocity we obtain from the wheel
            *  encoders is integrated over a different time interval than one
            *  loop-interval.  It's not clear which estimation technique would deliver
            *  a better result. For visualization purposes velocityK (in inches/sec)
            *  is in the standard human-readable form. Also of note, this variant
            *  doesn't include the gyro heading in its calculation.
            */
            final Twist2d velocity2 = Kinematics.forwardKinematics(
                                        mDrive.getLeftLinearVelocity(),
                                        mDrive.getRightLinearVelocity());
            final RobotStateMap.State lastState, nextState;
            lastState = mEncoderRobotState.getLatest();

            /* integrateForward: given a last state and a current velocity,
            *  estimate a new state (P2 = P1 + dPdt * dt)
            */
            nextState = Kinematics.integrateForwardKinematics(lastState, velocity1);

            /* record the new state estimate */
            mEncoderRobotState.addObservations(timestamp, nextState);
        }

        @Override
        public void onStop(double timestamp)
        {
            // no-op
        }
    }

    private Twist2d getVelocityFromDeltas(double leftEncoderDelta,
            double rightEncoderDelta, Rotation2d currentGyroAngle)
    {
        final Pose2d last_measurement = mEncoderRobotState.getLatestFieldToVehicle().getValue();
        return Kinematics.forwardKinematics(last_measurement.getRotation(), leftEncoderDelta,
                rightEncoderDelta, currentGyroAngle);
    }
}
