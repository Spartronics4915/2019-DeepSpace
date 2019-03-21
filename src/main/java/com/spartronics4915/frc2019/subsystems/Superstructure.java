package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.lib.util.ILooper;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.RobotStateMap;

import java.util.ArrayList;
import java.util.Optional;
import java.util.stream.Collectors;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.VisionUpdateManager;
import com.spartronics4915.frc2019.VisionUpdateManager.HeadingUpdate;
import com.spartronics4915.frc2019.VisionUpdateManager.PNPUpdate;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.geometry.Rotation2d;
import com.spartronics4915.lib.trajectory.TimedView;
import com.spartronics4915.lib.trajectory.TrajectoryIterator;
import com.spartronics4915.lib.trajectory.timing.TimedState;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.ILoop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The superstructure subsystem is the overarching superclass containing all
 * components of the superstructure: climber, harvester, and articulated
 * grabber, and lifter.
 *
 * The superstructure subsystem also contains some miscellaneous hardware that
 * is located in the superstructure but isn't part of any other subsystems like
 * the compressor, pressure sensor, and hopper wall pistons.
 *
 * Instead of interacting with subsystems like the feeder and intake directly,
 * the {@link Robot} class interacts with the superstructure, which passes on
 * the commands to the correct subsystem.
 *
 * The superstructure also coordinates actions between different subsystems like
 * the feeder and shooter.
 *
 * @see LED
 * @see Subsystem
 */
public class Superstructure extends Subsystem
{

    static Superstructure mInstance = null;

    public static Superstructure getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

    // Desired function from user
    public enum WantedState
    {
        // Regular driver control
        DRIVER_CONTROL,
        // Alignment and intake/place using vision
        ALIGN_AND_INTAKE_CARGO,
        ALIGN_AND_INTAKE_PANEL,
        ALIGN_AND_SHOOT_CARGO_ROCKET,
        ALIGN_AND_SHOOT_CARGO_BAY,
        ALIGN_AND_EJECT_PANEL,
        ALIGN_CLOSEST_REVERSE_TARGET,
        // Climb
        CLIMB,
        // Manual climb
        LOWER_CHUTE_AND_CLIMB,
        // Panel ejecting (no auto align)
        EJECT_PANEL,
        // Intaking
        INTAKE_CARGO,
        // Shooting cargo into bay
        SHOOT_CARGO_BAY,
    };

    // Internal state of the system
    public enum SystemState
    {
        /* Regular driver control */
        // All other states ingore driver input
        DRIVER_CONTROLLING,

        /* Climbing */
        LIFTING_TO_THREE,
        RUNNING_INTAKE_UNTIL_PLATFORM_CONTACT,
        RETRACTING_FORWARD_STRUTS,
        DRIVING_UNTIL_PLATFORM_FULL_SUPPORT,
        RETRACTING_REAR_STRUTS,

        /* Manual climb */
        LOWERING_CHUTE_AND_CLIMBING,
        // Then goes to LIFTING_TO_THREE

        /* Placing/intaking game pieces */
        // Alignment using vision+odometry (step 1)
        INTAKING_AND_ALIGNING_CLOSEST_FORWARD_TARGET, // This intakes cargo while driving
        ALIGNING_CLOSEST_REVERSE_TARGET,
        // Intaking (step 2)
        INTAKING_PANEL,
        // Placing (could also be step 2)
        MOVING_CHUTE_TO_EJECT_PANEL,
        EJECTING_PANEL,
        EJECTING_CARGO,
        // Backing out and turning (step 3, PANEL panels only)
        BACKING_OUT_FROM_LOADING,
        TURNING_AROUND,

        /* Intaking cargo */
        INTAKING_CARGO,

        /* Ejecting cargo into the bay after backing up */
        SHOOTING_CARGO_AND_BACKING,
    }

    // Superstructure doesn't own the drive, but needs to access it
    private final Drive mDrive = Drive.getInstance();
    private final CargoChute mCargoChute = CargoChute.getInstance();
    private final CargoIntake mCargoIntake = CargoIntake.getInstance();
    private final Climber mClimber = Climber.getInstance();
    private final PanelHandler mPanelHandler = PanelHandler.getInstance();

    private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private RobotStateMap mRobotStateMap = RobotStateEstimator.getInstance().getEncoderRobotStateMap();

    private static final double kPanelHandlingDuration = 0.3; // Seconds TODO: Tune me (also is this our responsibility?)

    private static final DriveSignal kPlatformDriveSpeed = new DriveSignal(1, 1);

    private static final Pose2d kBackOutOffset = new Pose2d(36, 0, Rotation2d.identity());

    // These constants are _just_ for dynamic paths. See TrajectoryGenerator for constants for premade paths
    // XXX: Currently unused
    private static final double kMaxPathVelocity = 240.0; // inches/s
    private static final double kMaxPathAccel = 48.0; // inches/s
    private static final double kMaxPathCentripetalAccel = 24.0; // inches/s
    private static final double kMaxPathVoltage = 9.0; // volts

    private WantedState mWantedState = WantedState.DRIVER_CONTROL;
    private SystemState mSystemState = SystemState.DRIVER_CONTROLLING;
    // We don't have a DRIVER_CONTROL_FORWARD and ..._REVERSE becase we need to persist driving direction across state changes
    private boolean mIsReversed = false;
    private boolean mGotVisionUpdate = false;

    private Superstructure()
    {
        logInitialized(true);
    }

    private ILoop mLoop = new ILoop()
    {

        private Timer mStateChangedTimer = new Timer();
        private boolean mStateChanged;

        @Override
        public void onStart(double timestamp)
        {
            synchronized (Superstructure.this)
            {
                mWantedState = WantedState.DRIVER_CONTROL;
                mSystemState = SystemState.DRIVER_CONTROLLING;
                mStateChangedTimer.reset();
                mStateChangedTimer.start();
                mStateChanged = true;

                updateCameraDirection();
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (Superstructure.this)
            {
                SystemState newState = defaultStateTransfer();
                switch (mSystemState)
                {
                    /* Regular driver control */
                    case DRIVER_CONTROLLING:
                        // Driver control doesn't really go through us, except for getDirectionMultiplier
                        break;

                    /* Climbing */
                    case LIFTING_TO_THREE:
                        mClimber.setWantedState(Climber.WantedState.CLIMB);
                        if (mClimber.atTarget() && newState == mSystemState)
                            newState = SystemState.RUNNING_INTAKE_UNTIL_PLATFORM_CONTACT;
                        break;
                    case RUNNING_INTAKE_UNTIL_PLATFORM_CONTACT:
                        mCargoIntake.setWantedState(CargoIntake.WantedState.CLIMB);
                        if (mClimber.atTarget() && newState == mSystemState)
                            newState = SystemState.RETRACTING_FORWARD_STRUTS;
                        break;
                    case RETRACTING_FORWARD_STRUTS:
                        mClimber.setWantedState(Climber.WantedState.RETRACT_FRONT_STRUTS);
                        if (mClimber.atTarget() && newState == mSystemState)
                            newState = SystemState.DRIVING_UNTIL_PLATFORM_FULL_SUPPORT;
                        break;
                    case DRIVING_UNTIL_PLATFORM_FULL_SUPPORT:
                        mDrive.setOpenLoop(kPlatformDriveSpeed);
                        if (mClimber.atTarget() && newState == mSystemState)
                            newState = SystemState.RETRACTING_REAR_STRUTS;
                        break;
                    case RETRACTING_REAR_STRUTS:
                        mClimber.setWantedState(Climber.WantedState.RETRACT_REAR_STRUTS);
                        mCargoIntake.setWantedState(CargoIntake.WantedState.HOLD);
                        mDrive.setOpenLoop(DriveSignal.BRAKE);

                        if (mWantedState == WantedState.CLIMB && mClimber.atTarget())
                        {
                            mWantedState = WantedState.DRIVER_CONTROL;
                            newState = SystemState.DRIVER_CONTROLLING;
                        }
                        break;

                    /* Manual climbing */
                    case LOWERING_CHUTE_AND_CLIMBING:
                        if (mStateChanged)
                            mCargoChute.setWantedState(CargoChute.WantedState.LOWER);
                        if (mCargoChute.atTarget() && mStateChangedTimer.hasPeriodPassed(Constants.kChuteLowRetractTime)
                                && mWantedState == WantedState.LOWER_CHUTE_AND_CLIMB)
                        {
                            mClimber.setWantedState(Climber.WantedState.CLIMB);
                            mWantedState = WantedState.DRIVER_CONTROL;
                            newState = SystemState.DRIVER_CONTROLLING;
                        }
                        break;

                    /* Placing/intaking game pieces */
                    case INTAKING_AND_ALIGNING_CLOSEST_FORWARD_TARGET:
                        mCargoIntake.setWantedState(CargoIntake.WantedState.INTAKE);
                        mCargoChute.setWantedState(CargoChute.WantedState.BRING_BALL_TO_TOP);
                        if (mStateChanged)
                            makeAndDrivePath(Pose2d.identity(), false);
                        // TODO: Target selection/vision integration
                        // VisionUpdateManager.forwardVisionManager.getLatestVisionUpdate()
                        //         .ifPresent(v -> makeAndDrivePath(v.getFieldPosition(mRobotStateMap)), false)

                        if (mWantedState == WantedState.ALIGN_AND_INTAKE_CARGO && (mDrive.isDoneWithTrajectory() || mCargoChute.atTarget()))
                        {
                            mCargoIntake.setWantedState(CargoIntake.WantedState.HOLD);
                            mCargoChute.setWantedState(CargoChute.WantedState.LOWER);
                            mDrive.setOpenLoop(DriveSignal.BRAKE);
                            mWantedState = WantedState.DRIVER_CONTROL;
                            newState = SystemState.DRIVER_CONTROLLING;
                        }
                        break;
                    case ALIGNING_CLOSEST_REVERSE_TARGET:
                        mCargoIntake.setWantedState(CargoIntake.WantedState.HOLD);
                        mCargoChute.setWantedState(CargoChute.WantedState.LOWER);
                        if (mStateChanged || !mGotVisionUpdate)
                        {
                            // Optional<HeadingUpdate> visionUpdate = VisionUpdateManager.reverseHeadingVisionManager.getLatestVisionUpdate();

                            // mGotVisionUpdate = visionUpdate.isPresent();
                            // visionUpdate.ifPresent(v -> mDrive.curveTowardsVisionTarget(v.getTargetInfo()));

                            Optional<PNPUpdate> visionUpdate = VisionUpdateManager.reversePNPVisionManager.getLatestVisionUpdate();

                            mGotVisionUpdate = visionUpdate.isPresent();
                            visionUpdate.ifPresent(
                                    v ->
                                    {
                                        makeAndDrivePath(v.getFieldPosition(mRobotStateMap).transformBy(-Constants.kRobotCenterToForward), true);

                                        dashboardPutString("TargetPose",
                                                v.getFieldPosition(mRobotStateMap).transformBy(-Constants.kRobotCenterToForward).toString());
                                        dashboardPutNumber("CapTime", v.frameCapturedTime);
                                    });
                        }

                        if (mDrive.isDoneWithTrajectory() && newState == mSystemState)
                        {
                            if (mWantedState == WantedState.ALIGN_AND_EJECT_PANEL)
                                newState = SystemState.MOVING_CHUTE_TO_EJECT_PANEL;
                            else if (mWantedState == WantedState.ALIGN_AND_INTAKE_PANEL)
                                newState = SystemState.INTAKING_PANEL;
                            else if (mWantedState == WantedState.ALIGN_AND_SHOOT_CARGO_BAY
                                    || mWantedState == WantedState.ALIGN_AND_SHOOT_CARGO_ROCKET)
                                newState = SystemState.EJECTING_CARGO;
                            else if (mWantedState == WantedState.ALIGN_CLOSEST_REVERSE_TARGET)
                            {
                                mWantedState = WantedState.DRIVER_CONTROL;
                                newState = SystemState.DRIVER_CONTROLLING;
                            }
                        }
                        break;
                    case INTAKING_PANEL:
                        mCargoChute.setWantedState(CargoChute.WantedState.LOWER);

                        if (newState == mSystemState && mStateChangedTimer.hasPeriodPassed(kPanelHandlingDuration))
                            newState = SystemState.BACKING_OUT_FROM_LOADING;
                        break;
                    case MOVING_CHUTE_TO_EJECT_PANEL:
                        mCargoChute.setWantedState(CargoChute.WantedState.LOWER);

                        if (newState == mSystemState && mStateChangedTimer.hasPeriodPassed(kPanelHandlingDuration) && mCargoChute.atTarget())
                            newState = SystemState.EJECTING_PANEL;
                        break;
                    case EJECTING_PANEL:
                        if (mStateChanged)
                        {
                            mCargoChute.setWantedState(CargoChute.WantedState.LOWER);
                            mPanelHandler.setWantedState(PanelHandler.WantedState.EJECT);
                        }

                        if ((mWantedState == WantedState.ALIGN_AND_EJECT_PANEL || mWantedState == WantedState.EJECT_PANEL)
                                && mCargoChute.atTarget() && mPanelHandler.atTarget())
                        {
                            mWantedState = WantedState.DRIVER_CONTROL;
                            newState = SystemState.DRIVER_CONTROLLING;
                        }
                        break;
                    case EJECTING_CARGO:
                        if (mWantedState == WantedState.ALIGN_AND_SHOOT_CARGO_BAY)
                        {
                            mCargoIntake.setWantedState(CargoIntake.WantedState.ARM_DOWN); // Brings arm down to avoid collision
                            mCargoChute.setWantedState(CargoChute.WantedState.SHOOT_BAY);
                        }
                        else if (mWantedState == WantedState.ALIGN_AND_SHOOT_CARGO_ROCKET)
                            mCargoChute.setWantedState(CargoChute.WantedState.SHOOT_ROCKET);
                        else
                            break;
                        if (mCargoChute.atTarget())
                        {
                            mWantedState = WantedState.DRIVER_CONTROL;
                            newState = SystemState.DRIVER_CONTROLLING;
                        }
                        break;
                    case BACKING_OUT_FROM_LOADING:
                        if (mStateChanged)
                            makeAndDrivePath(mRobotStateMap.getFieldToVehicle(Timer.getFPGATimestamp()).transformBy(kBackOutOffset), false);

                        if (newState == mSystemState && mDrive.isDoneWithTrajectory())
                            newState = SystemState.TURNING_AROUND;
                        break;
                    case TURNING_AROUND:
                        if (mStateChanged)
                            mDrive.setWantTurn(mDrive.getHeading().rotateBy(Rotation2d.fromDegrees(180)));

                        if (mWantedState == WantedState.ALIGN_AND_INTAKE_PANEL && mDrive.isDoneTurning())
                        {
                            mWantedState = WantedState.DRIVER_CONTROL;
                            newState = SystemState.DRIVER_CONTROLLING;
                        }
                        break;

                    /* Intaking cargo */
                    case INTAKING_CARGO:
                        if (mStateChanged)
                        {
                            mCargoIntake.setWantedState(CargoIntake.WantedState.INTAKE);
                            mCargoChute.setWantedState(CargoChute.WantedState.LOWER);
                            mCargoChute.setWantedState(CargoChute.WantedState.BRING_BALL_TO_TOP);
                        }

                        if (mCargoChute.atTarget())
                        {
                            mCargoIntake.setWantedState(CargoIntake.WantedState.HOLD);
                            mWantedState = WantedState.DRIVER_CONTROL;
                            newState = SystemState.DRIVER_CONTROLLING;
                        }
                        break;

                    /* Ejecting cargo into the bay after backing up */
                    case SHOOTING_CARGO_AND_BACKING:
                        if (mStateChanged)
                        {
                            makeAndDrivePath(
                                    mRobotStateMap.getFieldToVehicle(Timer.getFPGATimestamp()).transformBy(Constants.kShootIntoBayBackupDistance),
                                    false);
                            // mRobotStateMap.reset(Timer.getFPGATimestamp(), new Pose2d());
                            // mDrive.setHeading(Rotation2d.identity());
                            // mDrive.setTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().driveReverseToShootInBay);
                        }
                        else if (mDrive.isDoneWithTrajectory() && mWantedState == WantedState.SHOOT_CARGO_BAY)
                        {
                            mCargoChute.setWantedState(CargoChute.WantedState.SHOOT_BAY);
                            mWantedState = WantedState.DRIVER_CONTROL;
                            newState = SystemState.DRIVER_CONTROLLING;
                        }
                        break;
                    default:
                        logError("Unhandled system state!");
                        break;
                }

                if (newState != mSystemState)
                {
                    mStateChanged = true;
                    mStateChangedTimer.reset();
                    logNotice("System state to " + newState);
                }
                else
                    mStateChanged = false;

                mSystemState = newState;
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            stop();
            mGotVisionUpdate = false;
        }
    };

    private void makeAndDrivePath(Pose2d goalPose, boolean reversed)
    {
        try
        {
            ArrayList<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(mRobotStateMap.getFieldToVehicle(Timer.getFPGATimestamp()));
            waypoints.add(goalPose);

            // logNotice(waypoints.stream().map(Object::toString).collect(Collectors.joining(", ")));

            double startTime = Timer.getFPGATimestamp();
            TrajectoryIterator<TimedState<Pose2dWithCurvature>> t =
                    new TrajectoryIterator<>(new TimedView<>((mTrajectoryGenerator.generateTrajectory(reversed, waypoints))));
            // TODO: Maybe plug in our current velocity as the start velocity of the path?
            logNotice("Path generated; took " + (Timer.getFPGATimestamp() - startTime) + " seconds.");

            mDrive.setTrajectory(t);
        }
        catch (Exception e)
        {
            logException("Tried to drive an invalid path!", e);
        }
    }

    private SystemState defaultStateTransfer()
    {
        SystemState newState = mSystemState;
        switch (mWantedState)
        {
            case DRIVER_CONTROL:
                newState = SystemState.DRIVER_CONTROLLING;
                break;
            case ALIGN_AND_INTAKE_CARGO:
                // Assumes cargo is on the forward side
                if (mSystemState == SystemState.INTAKING_AND_ALIGNING_CLOSEST_FORWARD_TARGET)
                    break;
                newState = SystemState.INTAKING_AND_ALIGNING_CLOSEST_FORWARD_TARGET;
                break;
            case ALIGN_AND_INTAKE_PANEL:
                // Assumes PANEL intake is on the reverse side
                if (mSystemState == SystemState.ALIGNING_CLOSEST_REVERSE_TARGET ||
                        mSystemState == SystemState.INTAKING_PANEL ||
                        mSystemState == SystemState.BACKING_OUT_FROM_LOADING ||
                        mSystemState == SystemState.TURNING_AROUND)
                    break;
                newState = SystemState.ALIGNING_CLOSEST_REVERSE_TARGET;
                break;
            case ALIGN_AND_SHOOT_CARGO_BAY:
                if (isInAlignAndEjectCargoSystemState())
                    break;
                newState = SystemState.ALIGNING_CLOSEST_REVERSE_TARGET;
                break;
            case ALIGN_AND_SHOOT_CARGO_ROCKET:
                if (isInAlignAndEjectCargoSystemState())
                    break;
                newState = SystemState.ALIGNING_CLOSEST_REVERSE_TARGET;
                break;
            case ALIGN_AND_EJECT_PANEL:
                if (mSystemState == SystemState.ALIGNING_CLOSEST_REVERSE_TARGET ||
                        mSystemState == SystemState.MOVING_CHUTE_TO_EJECT_PANEL ||
                        mSystemState == SystemState.EJECTING_PANEL)
                    break;
                newState = SystemState.ALIGNING_CLOSEST_REVERSE_TARGET;
                break;
            case ALIGN_CLOSEST_REVERSE_TARGET:
                if (mSystemState == SystemState.ALIGNING_CLOSEST_REVERSE_TARGET)
                    break;
                newState = SystemState.ALIGNING_CLOSEST_REVERSE_TARGET;
                break;
            case CLIMB:
                if (mSystemState == SystemState.LIFTING_TO_THREE ||
                        mSystemState == SystemState.RUNNING_INTAKE_UNTIL_PLATFORM_CONTACT ||
                        mSystemState == SystemState.RETRACTING_FORWARD_STRUTS ||
                        mSystemState == SystemState.DRIVING_UNTIL_PLATFORM_FULL_SUPPORT ||
                        mSystemState == SystemState.RETRACTING_REAR_STRUTS)
                    break;
                newState = SystemState.LIFTING_TO_THREE;
                break;
            case LOWER_CHUTE_AND_CLIMB:
                newState = SystemState.LOWERING_CHUTE_AND_CLIMBING;
                break;
            case EJECT_PANEL:
                if (mSystemState == SystemState.MOVING_CHUTE_TO_EJECT_PANEL || mSystemState == SystemState.EJECTING_PANEL)
                    break;
                newState = SystemState.MOVING_CHUTE_TO_EJECT_PANEL;
                break;
            case INTAKE_CARGO:
                if (mSystemState == SystemState.INTAKING_CARGO)
                    break;
                newState = SystemState.INTAKING_CARGO;
                break;
            case SHOOT_CARGO_BAY:
                if (mSystemState == SystemState.SHOOTING_CARGO_AND_BACKING)
                    break;
                newState = SystemState.SHOOTING_CARGO_AND_BACKING;
                break;
            default:
                logError("Unhandled wanted state in default state transfer!");
                newState = SystemState.DRIVER_CONTROLLING;
                break;
        }
        return newState;
    }

    private boolean isInAlignAndEjectCargoSystemState()
    {
        return mSystemState == SystemState.INTAKING_AND_ALIGNING_CLOSEST_FORWARD_TARGET ||
                mSystemState == SystemState.EJECTING_CARGO;
    }

    public synchronized void setWantedState(WantedState wantedState)
    {
        logNotice("Wanted state to " + wantedState.toString());
        mWantedState = wantedState;
    }

    public synchronized void reverseDrivingDirection()
    {
        mIsReversed = !mIsReversed;
        updateCameraDirection();

    }

    public synchronized boolean isDrivingReversed()
    {
        return mIsReversed;
    }

    public synchronized boolean isDriverControlled()
    {
        return mWantedState == WantedState.INTAKE_CARGO || mWantedState == WantedState.EJECT_PANEL || mWantedState == WantedState.DRIVER_CONTROL
                || mWantedState == WantedState.LOWER_CHUTE_AND_CLIMB;
    }

    private void updateCameraDirection()
    {
        SmartDashboard.putString("Driver/VideoStream", mIsReversed ? "Back" : "Front");
    }

    @Override
    public void stop()
    {
        // Subsystem manager stops these, we don't
        mWantedState = WantedState.DRIVER_CONTROL;
        mSystemState = SystemState.DRIVER_CONTROLLING;
    }

    @Override
    public void zeroSensors()
    {

    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper)
    {
        enabledLooper.register(mLoop);
    }

    @Override
    public boolean checkSystem(String subsystem)
    {
        logWarning("checkSystem not implemented");
        return false;
    }

    @Override
    public void outputTelemetry()
    {
        dashboardPutState(mSystemState.toString());
        dashboardPutWantedState(mWantedState.toString());
        dashboardPutBoolean("Reverse", mIsReversed);
    }
}
