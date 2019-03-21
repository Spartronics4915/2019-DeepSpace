package com.spartronics4915.frc2019;

import com.spartronics4915.frc2019.auto.AutoModeExecutor;
import com.spartronics4915.frc2019.loops.Looper;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator;
import com.spartronics4915.frc2019.subsystems.*;
import com.spartronics4915.frc2019.subsystems.CargoChute.WantedState;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.geometry.Rotation2d;
import com.spartronics4915.lib.geometry.Twist2d;
import com.spartronics4915.lib.physics.DifferentialDrive;
import com.spartronics4915.lib.physics.DifferentialDrive.ChassisState;
import com.spartronics4915.lib.physics.DifferentialDrive.DriveDynamics;
import com.spartronics4915.lib.physics.DifferentialDrive.WheelState;
import com.spartronics4915.lib.util.*;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

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
    private CargoChute mCargoChute = null;
    private CargoIntake mCargoIntake = null;
    private Climber mClimber = null;
    private LED mLED = null;
    private AnalogInput mPressureSensor = null;
    private RobotStateEstimator mRobotStateEstimator = null;
    private Superstructure mSuperstructure = null;
    private AutoModeExecutor mAutoModeExecutor;
    private Timer mCodeTimer = new Timer();
    // private PowerDistributionPanel mPDP = new PowerDistributionPanel(); FIXME
    private double mNextReportDue = 0.0; // see outputToSmartDashboard
    private double mLastTeleopLoopTime; // Seconds
    private ChassisState mLastTeleopVelocity = new ChassisState(); // rad/s and rad/s^2

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
                mCargoChute = CargoChute.getInstance();
                mCargoIntake = CargoIntake.getInstance();
                mClimber = Climber.getInstance();
                mLED = LED.getInstance();
                mPressureSensor = new AnalogInput(1);
                mSuperstructure = Superstructure.getInstance();
                mRobotStateEstimator = RobotStateEstimator.getInstance();

                mSubsystemManager = new SubsystemManager(
                        Arrays.asList(
                                mRobotStateEstimator,
                                mDrive,
                                mPanelHandler,
                                mCargoChute,
                                mCargoIntake,
                                mClimber,
                                mLED,
                                mSuperstructure));
                mSubsystemManager.registerEnabledLoops(mEnabledLooper);
                mSubsystemManager.registerDisabledLoops(mDisabledLooper);
                SmartDashboard.putString(kRobotTestModeOptions,
                        "None,CargoChute,CargoIntake,Climber,PanelHandler,Drive,All");
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
            mRobotStateEstimator.resetRobotStateMaps();

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

            mRobotStateEstimator.resetRobotStateMaps();
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

            mLastTeleopLoopTime = Timer.getFPGATimestamp();
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
            if (testMode.equals("CargoChute") || testMode.equals("All"))
            {
                success &= mCargoChute.checkSystem(testVariant);
            }
            if (testMode.equals("CargoIntake") || testMode.equals("All"))
            {
                success &= mCargoIntake.checkSystem(testVariant);
            }
            if (testMode.equals("Climber") || testMode.equals("All"))
            {
                success &= mClimber.checkSystem(testVariant);
            }
            if (testMode.equals("PanelHandler") || testMode.equals("All"))
            {
                success &= mPanelHandler.checkSystem(testVariant);
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
            VisionUpdateManager.reversePNPVisionManager.clearVisionUpdate();
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
        double[] codeTimes = new double[20]; // for diagnosing loop time consumption
        int nctr = 0;
        mCodeTimer.reset();
        mCodeTimer.start();
        try
        {
            if (mSuperstructure.isDriverControlled())
            {
                DriveSignal command = ArcadeDriveHelper.arcadeDrive(mControlBoard.getThrottle() * (mSuperstructure.isDrivingReversed() ? -1 : 1),
                        mControlBoard.getTurn(),
                        mControlBoard.getSlowMode());

                codeTimes[nctr++] = mCodeTimer.get(); // 0 after arcadeDrive

                // double throttle = mControlBoard.getThrottle() * (mSuperstructure.isDrivingReversed() ? -1 : 1);
                // throttle = Math.copySign(Math.pow(Math.abs(throttle), 5.0/2.0), throttle) * Constants.kTeleopMaxChassisVel;

                // double turn = mControlBoard.getTurn() * (mSuperstructure.isDrivingReversed() ? -1 : 1);
                // turn = Math.copySign(Math.pow(Math.abs(turn), 5.0/3.0), turn) * Constants.kTeleopMaxChassisVel;

                // double dt = Timer.getFPGATimestamp() - mLastTeleopLoopTime;
                // ChassisState vel = new ChassisState(throttle, turn);
                // ChassisState accel = new ChassisState((throttle - mLastTeleopVelocity.linear) / dt, (turn - mLastTeleopVelocity.angular) / dt);

                // mDrive.setVelocityForChassisState(vel, accel);

                // mLastTeleopLoopTime = Timer.getFPGATimestamp();
                // mLastTeleopVelocity = new ChassisState(mDrive.getLinearVelocity(), mDrive.getLinearVelocity());

                mDrive.setOpenLoop(command);
                // mDrive.setVelocity(command, new DriveSignal(
                //     command.scale(Constants.kDriveLeftKv * (Constants.kDriveWheelDiameterInches / 2)).getLeft() + Math.copySign(Constants.kDriveLeftVIntercept, command.getLeft()),
                //     command.scale(Constants.kDriveRightKv * (Constants.kDriveWheelDiameterInches / 2)).getRight() + Math.copySign(Constants.kDriveRightVIntercept, command.getRight())
                // )); XXX Conversions on Kv are wrong

                codeTimes[nctr++] = mCodeTimer.get(); // 1 after setOpenLoop

                // Button Board ----------------------------------------------------------
                mControlBoard.updatePOV();

                // CLIMBING
                if (mControlBoard.getClimb())
                    mSuperstructure.setWantedState(Superstructure.WantedState.LOWER_CHUTE_AND_CLIMB);

                codeTimes[nctr++] = mCodeTimer.get(); // 2 after climbing

                // INTAKE
                if (mControlBoard.getAssistedIntakeCargo())
                    mSuperstructure.setWantedState(Superstructure.WantedState.ALIGN_AND_INTAKE_CARGO);
                else if (mControlBoard.getGroundEjectCargo())
                {
                    mCargoIntake.setWantedState(CargoIntake.WantedState.EJECT);
                    mCargoChute.setWantedState(CargoChute.WantedState.EJECT_BACK);
                }
                else if (mControlBoard.getManualIntakeCargo())
                {
                    mSuperstructure.setWantedState(Superstructure.WantedState.INTAKE_CARGO);
                }
                codeTimes[nctr++] = mCodeTimer.get(); // 3 after intake

                // CARGO RAMP
                if (mControlBoard.getManualRamp())
                {
                    if (!mCargoChute.isRampRunning())
                        mCargoChute.setWantedState(CargoChute.WantedState.RAMP_MANUAL);
                    else
                        mCargoChute.setWantedState(CargoChute.WantedState.HOLD_MANUAL);
                }
                else if (mControlBoard.getAssistedShootRocket())
                    mSuperstructure.setWantedState(Superstructure.WantedState.ALIGN_AND_SHOOT_CARGO_ROCKET);
                else if (mControlBoard.getAssistedShootBay())
                    mSuperstructure.setWantedState(Superstructure.WantedState.ALIGN_AND_SHOOT_CARGO_BAY);
                else if (mControlBoard.getSelectLeftVisionTarget())
                {
                    //TODO: add this functionality
                }
                else if (mControlBoard.getSelectRightVisionTarget())
                {
                    //TODO: add this functionality
                }
                else if (mControlBoard.getManualShootCargoBay())
                    mSuperstructure.setWantedState(Superstructure.WantedState.SHOOT_CARGO_BAY);
                    // mCargoChute.setWantedState(CargoChute.WantedState.SHOOT_BAY);
                else if (mControlBoard.getManualShootCargoRocket())
                    mCargoChute.setWantedState(CargoChute.WantedState.SHOOT_ROCKET);
                else if (mControlBoard.getManualChuteUp())
                    mCargoChute.setWantedState(CargoChute.WantedState.RAISE);
                else if (mControlBoard.getManualChuteDown())
                    mCargoChute.setWantedState(CargoChute.WantedState.LOWER);

                codeTimes[nctr++] = mCodeTimer.get(); // 4 after cargochute

                // PANEL HANDLER
                if (mControlBoard.getAssistedIntakePanel())
                    mSuperstructure.setWantedState(Superstructure.WantedState.ALIGN_AND_INTAKE_PANEL);
                else if (mControlBoard.getAssistedEjectPanel())
                    mSuperstructure.setWantedState(Superstructure.WantedState.ALIGN_AND_EJECT_PANEL);
                else if (mControlBoard.getManualEjectPanel())
                    mSuperstructure.setWantedState(Superstructure.WantedState.EJECT_PANEL);

                codeTimes[nctr++] = mCodeTimer.get(); // 5 after panelhandler

                // EVERYTHING
                if (mControlBoard.getInsideFramePerimeter())
                {
                    mCargoChute.setWantedState(CargoChute.WantedState.LOWER);
                    mCargoIntake.setWantedState(CargoIntake.WantedState.HOLD);
                }
                if (mControlBoard.getTestButtonOne()) // 2: 5
                {
                    // mCargoIntake.setWantedState(CargoIntake.WantedState.HOLD);
                }
                if (mControlBoard.getTestButtonTwo())
                {
                    // mCargoIntake.setWantedState(CargoIntake.WantedState.CLIMB);
                }
                if (mControlBoard.getTestButtonThree()) // 2: 7
                {
                    // mCargoIntake.setWantedState(CargoIntake.WantedState.EJECT);
                }
                if (mControlBoard.getChangeSelectedVisionIndex())
                {
                    int selectedIndex = (int) SmartDashboard.getNumber(Constants.kVisionSelectedIndexKey, -1);
                    if (++selectedIndex >= Constants.kMaxVisionTargets)
                        selectedIndex = 0;
                    SmartDashboard.putNumber(Constants.kVisionSelectedIndexKey, selectedIndex);
                }
                codeTimes[nctr++] = mCodeTimer.get(); // 6 after subsystems

                //TEST BUTTONBOARD
                if (mControlBoard.getClimbExtendAllPneumatics())
                {
                    mSuperstructure.setWantedState(Superstructure.WantedState.LOWER_CHUTE_AND_CLIMB);
                }
                else if (mControlBoard.getClimbIntake())
                {
                    mCargoIntake.setWantedState(CargoIntake.WantedState.CLIMB);
                }
                else if (mControlBoard.getClimbRetractFrontPneumatics())
                {
                    mClimber.setWantedState(Climber.WantedState.RETRACT_FRONT_STRUTS);
                }
                else if (mControlBoard.getClimbRetractBackPneumatics())
                {
                    mClimber.setWantedState(Climber.WantedState.RETRACT_REAR_STRUTS);
                }
                else if (mControlBoard.getIntakeArmDown())
                {
                    mCargoIntake.setWantedState(CargoIntake.WantedState.ARM_DOWN);
                }
                else if (mControlBoard.getIntakeHold())
                {
                    mCargoIntake.setWantedState(CargoIntake.WantedState.HOLD);
                    mCargoChute.setWantedState(CargoChute.WantedState.HOLD_MANUAL);
                }
                else if (mControlBoard.getIntakeStopMotors())
                {
                    mCargoIntake.setWantedState(CargoIntake.WantedState.MOTORS_STOP);
                }
                codeTimes[nctr++] = mCodeTimer.get(); // 7 after testbuttonboard

                //Driver Joystick-----------------------------------------------------------
                if (mControlBoard.getReverseDirection())
                    mSuperstructure.reverseDrivingDirection();

                if (mControlBoard.getReturnToDriverControl())
                    mSuperstructure.setWantedState(Superstructure.WantedState.ALIGN_CLOSEST_REVERSE_TARGET);
            }
            else if (mControlBoard.getReturnToDriverControl())
                mSuperstructure.setWantedState(Superstructure.WantedState.DRIVER_CONTROL);

            codeTimes[nctr++] = mCodeTimer.get(); // 8 at end
        }
        catch (Throwable t)
        {
            Logger.logThrowableCrash(t);
            throw t;
        }

        outputToSmartDashboard();
        codeTimes[nctr++] = mCodeTimer.get(); // 9 after telemetry
        double loopTime = codeTimes[nctr - 1];
        if (loopTime > .025)
        {
            String str = "looptime overrun, offenders:\n";
            for (int i = 0; i < nctr; i++)
            {
                str += "  " + i + " " + codeTimes[i] + "\n";
            }
            if (loopTime > .1)
                Logger.notice("BIG " + str);
            else
                Logger.debug(str);
        }
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
        /*
         * need to constrain the amount of network table traffic we
         * produce each loop:
         * - outputTelemetry supports round-robin distribution of telemetry
         * - slowly varying data is "automatically filtered" by network tables
         * - but we impose an update rate max on less important data
         * NB: it's possible that slow/variable times is actually the result
         * of multithreaded synchronization locks.
         */
        mEnabledLooper.outputToSmartDashboard(); // outputs _dt
        mSubsystemManager.outputToTelemetry(true/* round-robin */);

        double now = Timer.getFPGATimestamp();
        if (now > this.mNextReportDue)
        {
            this.mNextReportDue = now + 1.0; // once per second
            // nb: MatchTime is approximate
            SmartDashboard.putNumber("DriverStation/MatchTime",
                    DriverStation.getInstance().getMatchTime());
            SmartDashboard.putNumber("Robot/BatteryVoltage",
                    RobotController.getBatteryVoltage());
            SmartDashboard.putNumber("Robot/Pressure", 250 * mPressureSensor.getVoltage() / 5.0 - 25);
            // SmartDashboard.putNumber("Robot/BatteryCurrent",
            //         mPDP.getTotalCurrent()); XXX: Spews CAN errors
        }
    }
}
