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
    private double mLeftEncoderPrevDistance = 0.0;
    private double mRightEncoderPrevDistance = 0.0;

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
            mLeftEncoderPrevDistance = mDrive.getLeftEncoderDistance();
            mRightEncoderPrevDistance = mDrive.getRightEncoderDistance();

        }

        @Override
        public synchronized void onLoop(double timestamp)
        {
            final double leftDistance = mDrive.getLeftEncoderDistance();
            final double rightDistance = mDrive.getRightEncoderDistance();
            final double leftDelta = leftDistance - mLeftEncoderPrevDistance;
            final double rightDelta = rightDistance - mRightEncoderPrevDistance;
            final Rotation2d gyroAngle = mDrive.getHeading();

            mLeftEncoderPrevDistance = leftDistance;
            mRightEncoderPrevDistance = rightDistance;

            final Twist2d measured_velocity = getVelocityFromDeltas(
                    leftDelta, rightDelta, gyroAngle);
            final Twist2d predicted_velocity = Kinematics.forwardKinematics(mDrive.getLeftLinearVelocity(),
                    mDrive.getRightLinearVelocity());

            mEncoderRobotState.addObservations(timestamp,
                Kinematics.integrateForwardKinematics(mEncoderRobotState.getLatestFieldToVehicle().getValue(), measured_velocity),
                measured_velocity, predicted_velocity);
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
