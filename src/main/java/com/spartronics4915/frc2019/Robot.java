package com.spartronics4915.frc2019;

import com.spartronics4915.frc2019.auto.AutoModeExecutor;
import com.spartronics4915.frc2019.lidar.LidarProcessor;
import com.spartronics4915.frc2019.lidar.LidarServer;
import com.spartronics4915.frc2019.loops.Looper;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator;
import com.spartronics4915.frc2019.subsystems.*;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.util.*;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;

public class Robot extends IterativeRobot {
    private Looper mEnabledLooper = new Looper();
    private Looper mDisabledLooper = new Looper();
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    private IControlBoard mControlBoard = ControlBoard.getInstance();
    private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private final SubsystemManager mSubsystemManager = new SubsystemManager(
            Arrays.asList(
                    RobotStateEstimator.getInstance(),
                    Drive.getInstance(),
                    Superstructure.getInstance()
            )
    );

    private Drive mDrive = Drive.getInstance();

    private AutoModeExecutor mAutoModeExecutor;

    public Robot() {
        Logger.logRobotConstruction();
    }

    @Override
    public void robotInit() {
        try {
            //init camera stream
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 15);
            MjpegServer cameraServer = new MjpegServer("serve_USB Camera 0", Constants.kCameraStreamPort);
            cameraServer.setSource(camera);

            Logger.logRobotInit();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            mEnabledLooper.register(LidarProcessor.getInstance());

            try {
                SmartDashboard.putString("LIDAR status", "starting");
                boolean started = LidarServer.getInstance().start();
                SmartDashboard.putString("LIDAR status", started ? "started" : "failed to start");
            } catch (Throwable t) {
                SmartDashboard.putString("LIDAR status", "crashed: " + t);
                t.printStackTrace();
                throw t;
            }

            AutoModeSelector.updateSmartDashboard();

            mTrajectoryGenerator.generateTrajectories();
        } catch (Throwable t) {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        SmartDashboard.putString("Match Cycle", "DISABLED");

        try {
            Logger.logDisabledInit();
            mEnabledLooper.stop();
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            Drive.getInstance().zeroSensors();
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            // Reset all auto mode state.
            mAutoModeExecutor = new AutoModeExecutor();

            mDisabledLooper.start();
        } catch (Throwable t) {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        try {
            Logger.logAutoInit();
            mDisabledLooper.stop();

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            Drive.getInstance().zeroSensors();

            mAutoModeExecutor.setAutoMode(AutoModeSelector.getSelectedAutoMode());
            mAutoModeExecutor.start();

            mEnabledLooper.start();
        } catch (Throwable t) {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        SmartDashboard.putString("Match Cycle", "TELEOP");

        try {
            Logger.logTeleopInit();
            mDisabledLooper.stop();
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mEnabledLooper.start();

            mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL); // Reset velocity setpoints
            mDrive.setOpenLoop(new DriveSignal(0.05, 0.05));
        } catch (Throwable t) {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        SmartDashboard.putString("Match Cycle", "TEST");

        try {
            System.out.println("Starting check systems.");

            mDisabledLooper.stop();
            mEnabledLooper.stop();

            // Call check system methods here

        } catch (Throwable t) {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        SmartDashboard.putString("Match Cycle", "DISABLED");

        try {
            outputToSmartDashboard();

        } catch (Throwable t) {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        outputToSmartDashboard();
        try {

        } catch (Throwable t) {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putString("Match Cycle", "TELEOP");
        double timestamp = Timer.getFPGATimestamp();

        double throttle = mControlBoard.getThrottle();
        double turn = mControlBoard.getTurn();

        try {
            mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn(),
                    mDrive.isHighGear()));

            outputToSmartDashboard();
        } catch (Throwable t) {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {
        SmartDashboard.putString("Match Cycle", "TEST");
    }

    public void outputToSmartDashboard() {
        RobotState.getInstance().outputToSmartDashboard();
        mSubsystemManager.outputToTelemetry();
        mEnabledLooper.outputToSmartDashboard();
    }
}
