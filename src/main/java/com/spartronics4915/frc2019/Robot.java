package com.spartronics4915.frc2019;

import com.spartronics4915.frc2019.auto.AutoModeExecutor;
import com.spartronics4915.frc2019.loops.Looper;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator;
import com.spartronics4915.frc2019.subsystems.*;
import com.spartronics4915.lib.util.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.io.InputStream;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.jar.Attributes;
import java.util.jar.Manifest;

public class Robot extends TimedRobot
{
    private Looper mEnabledLooper = new Looper();
    private Looper mDisabledLooper = new Looper();
    private IControlBoard mControlBoard = null;
    private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private SubsystemManager mSubsystemManager = null;
    private Drive mDrive = null;
    private PanelHandler mPanelHandler = null;
    private CargoHandler mCargoHandler = null;
    private CargoIntake mCargoIntake = null;
    private Climber mClimber = null;
    private LED mLED = null;
    private Superstructure mSuperstructure = null;
    private AutoModeExecutor mAutoModeExecutor;

    // smartdashboard keys
    private static final String kRobotLogVerbosity = "Robot/Verbosity";
    private static final String kRobotTestModeOptions = "TestModeOptions";
    private static final String kRobotTestMode = "TestMode";
    private static final String kRobotTestVariant = "TestVariant";

    public Robot()
    {
        Logger.logRobotConstruction();
    }

    @Override
    public void robotInit()
    {
        try
        {
            SmartDashboard.putString("Robot/GamePhase", "ROBOT INIT");
            Logger.logRobotInit();

            try (InputStream manifest =
                    getClass().getClassLoader().getResourceAsStream("META-INF/MANIFEST.MF"))
            {
                // build a version string
                Attributes attributes = new Manifest(manifest).getMainAttributes();
                String buildStr = "by: " + attributes.getValue("Built-By") +
                        "  on: " + attributes.getValue("Built-At") +
                        "  (" + attributes.getValue("Code-Version") + ")";
                SmartDashboard.putString("Build", buildStr);
                SmartDashboard.putString(kRobotLogVerbosity, "DEBUG"); // Verbosity level

                Logger.notice("=================================================");
                Logger.notice(Instant.now().toString());
                Logger.notice("Built " + buildStr);
                Logger.notice("=================================================");

            }
            catch (IOException e)
            {
                SmartDashboard.putString("Build", "version not found!");
                Logger.warning("Build version not found!");
                DriverStation.reportError(e.getMessage(), false);
            }

            // We should CANProbe before subsystems, because
            // they may invoke CANProbe validation methods.
            CANProbe canProbe = CANProbe.getInstance();
            ArrayList<String> canReport = canProbe.getReport();
            Logger.notice("CANDevicesFound: " + canReport);
            int numDevices = canProbe.getCANDeviceCount();
            SmartDashboard.putString("CANBusStatus",
                    numDevices == Constants.kNumCANDevices ? "OK"
                            : ("" + numDevices + "/" + Constants.kNumCANDevices));

            try
            {
                mDrive = Drive.getInstance();
                mPanelHandler = PanelHandler.getInstance();
                mCargoHandler = CargoHandler.getInstance();
                mCargoIntake = CargoIntake.getInstance();
                mClimber = Climber.getInstance();
                mLED = LED.getInstance();
                mSuperstructure = Superstructure.getInstance();
                mControlBoard = new ControlBoard();

                mSubsystemManager = new SubsystemManager(
                        Arrays.asList(
                                RobotStateEstimator.getInstance(),
                                mDrive,
                                mPanelHandler,
                                mCargoHandler,
                                mCargoIntake,
                                mClimber,
                                mLED,
                                mSuperstructure));
                mSubsystemManager.registerEnabledLoops(mEnabledLooper);
                mSubsystemManager.registerDisabledLoops(mDisabledLooper);
                SmartDashboard.putString(kRobotTestModeOptions, 
                                         "None,Drive,All");
                SmartDashboard.putString(kRobotTestMode, "None");
                SmartDashboard.putString(kRobotTestVariant, "");

                mControlBoard = new ControlBoard();
            }
            catch (Exception e)
            {
                // Try to avoid "robots don't quit" if there's a bug in subsystem init
                Logger.logThrowableCrash("ERROR Couldn't instantiate subsystems", e);
            }

            AutoModeSelector.updateSmartDashboard();
            Logger.debug("Generating trajectories...");
            mTrajectoryGenerator.generateTrajectories();
        }
        catch (Throwable t)
        {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit()
    {
        SmartDashboard.putString("Robot/GamePhase", "DISABLED");
        try
        {
            Logger.logDisabledInit();
            Logger.setVerbosity(SmartDashboard.getString(kRobotLogVerbosity, "DEBUG"));

            mEnabledLooper.stop();
            if (mAutoModeExecutor != null)
            {
                mAutoModeExecutor.stop();
            }

            Drive.getInstance().zeroSensors();
            RobotStateEstimator.getInstance().resetRobotStateMaps(Timer.getFPGATimestamp());

            // Reset all auto mode state.
            mAutoModeExecutor = new AutoModeExecutor();

            mDisabledLooper.start();
        }
        catch (Throwable t)
        {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit()
    {
        SmartDashboard.putString("Robot/GamePhase", "AUTONOMOUS");

        try
        {
            Logger.logAutoInit();
            Logger.setVerbosity(SmartDashboard.getString(kRobotLogVerbosity, "NOTICE"));

            mDisabledLooper.stop();

            RobotStateEstimator.getInstance().resetRobotStateMaps(Timer.getFPGATimestamp());
            Drive.getInstance().zeroSensors();

            mAutoModeExecutor.setAutoMode(AutoModeSelector.getSelectedAutoMode());
            mAutoModeExecutor.start();

            mEnabledLooper.start();
        }
        catch (Throwable t)
        {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit()
    {
        SmartDashboard.putString("Robot/GamePhase", "TELEOP");

        try
        {
            Logger.logTeleopInit();
            Logger.setVerbosity(SmartDashboard.getString(kRobotLogVerbosity, "NOTICE"));

            mDisabledLooper.stop();
            if (mAutoModeExecutor != null)
            {
                mAutoModeExecutor.stop();
            }

            // RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity()); Do not do this here 
            mEnabledLooper.start();

            mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL); // Reset velocity setpoints
            mDrive.setOpenLoop(new DriveSignal(0.05, 0.05));
        }
        catch (Throwable t)
        {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit()
    {
        SmartDashboard.putString("Robot/GamePhase", "TEST");
        Logger.setVerbosity(SmartDashboard.getString(kRobotLogVerbosity, "DEBUG"));

        try
        {
            Logger.logTestInit();

            mDisabledLooper.stop();
            mEnabledLooper.start();

            String testMode = SmartDashboard.getString(kRobotTestMode, "None");
            String testVariant = SmartDashboard.getString(kRobotTestVariant, "");
            if (testMode.equals("None"))
            {
                Logger.notice("Robot: no tests to run");
                mEnabledLooper.stop();
                return;
            }
            else
            {
                Logger.notice("Robot: running test mode " + testMode +
                        " variant:" + testVariant + " -------------------------");
            }
            Logger.notice("Waiting 5 seconds before running test methods.");
            Timer.delay(5);

            boolean success = true;
            if (testMode.equals("Drive") || testMode.equals("All"))
            {
                success &= mDrive.checkSystem(testVariant);
            }

            if (!success)
            {
                Logger.error("Robot: CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
            }
            else
            {
                Logger.notice("Robot: ALL SYSTEMS PASSED");
            }
        }
        catch (Throwable t)
        {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic()
    {
        try
        {
            outputToSmartDashboard();

        }
        catch (Throwable t)
        {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic()
    {
        try
        {
            if (mControlBoard.getReturnToDriverControl() && mAutoModeExecutor.getAutoMode().isActive())
                mAutoModeExecutor.stop(); // Careful! Teleop init doesn't get called until teleop actually starts
            else if (!mAutoModeExecutor.getAutoMode().isActive())
                teleopPeriodic();

            outputToSmartDashboard();
        }
        catch (Throwable t)
        {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic()
    {
        SmartDashboard.putString("Robot/GamePhase", "TELEOP");
        double timestamp = Timer.getFPGATimestamp();
        double throttle = mControlBoard.getThrottle();
        double turn = mControlBoard.getTurn();

        try
        {
            if (mSuperstructure.isDriverControlled())
            {
                DriveSignal command = ArcadeDriveHelper.arcadeDrive(mControlBoard.getThrottle(), mControlBoard.getTurn(),
                    true /* TODO: Decide squared inputs or not */).scale(mSuperstructure.isDrivingReversed() ? -1 : 1);

                // mDrive.setOpenLoop(command);
                mDrive.setVelocity(command, new DriveSignal(
                    command.scale(Constants.kDriveLeftKv).getLeft() + Math.copySign(Constants.kDriveLeftVIntercept, command.getLeft()),
                    command.scale(Constants.kDriveLeftKv).getRight() + Math.copySign(Constants.kDriveLeftVIntercept, command.getRight())
                ));

                if (mControlBoard.getReverseDirection())
                {
                    mSuperstructure.reverseDrivingDirection();
                }
                else if (mControlBoard.getDriveToSelectedTarget())
                {
                    mSuperstructure.setWantedState(Superstructure.WantedState.ALIGN_AND_INTAKE_CARGO);
                }
                // TODO (for button person): add buttons for all superstructure wanted states
            }
            else if (mControlBoard.getReturnToDriverControl())
            {
                mSuperstructure.setWantedState(Superstructure.WantedState.DRIVER_CONTROL);
            }
        }
        catch (Throwable t)
        {
            Logger.logThrowableCrash(t);
            throw t;
        }

        outputToSmartDashboard();
    }

    @Override
    public void testPeriodic()
    {
        outputToSmartDashboard();
    }

     /**
     * Unused but required function. Plays a similar role to our
     * allPeriodic method. Presumably the timing in IterativeRobotBase wasn't
     * to the liking of initial designers of this system. Perhaps because
     * we don't want it to run during testPeriodic.
     */
    @Override
    public void robotPeriodic()
    {
        // intentionally left blank
    }

    public void outputToSmartDashboard()
    {
        mSubsystemManager.outputToTelemetry();
        mEnabledLooper.outputToSmartDashboard();
        SmartDashboard.putNumber("Robot/BatteryVoltage", 
            RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Robot/InputCurrent", 
            RobotController.getInputCurrent());
    }
}
