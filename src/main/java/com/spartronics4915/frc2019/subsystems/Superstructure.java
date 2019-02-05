package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.lib.util.ILooper;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.RobotStateMap;

import java.util.ArrayList;
import java.util.Arrays;

import com.spartronics4915.frc2019.paths.TrajectoryGenerator;
import com.spartronics4915.frc2019.planners.DriveMotionPlanner;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.geometry.Rotation2d;
import com.spartronics4915.lib.trajectory.TimedView;
import com.spartronics4915.lib.trajectory.Trajectory;
import com.spartronics4915.lib.trajectory.TrajectoryIterator;
import com.spartronics4915.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.spartronics4915.lib.trajectory.timing.TimedState;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.ILoop;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;

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
        ALIGN_AND_EJECT_CARGO,
        ALIGN_AND_EJECT_PANEL,
        // Climb
        CLIMB,
        // Panel ejecting (no auto align)
        EJECT_PANEL,
    };

    // Internal state of the system
    public enum SystemState
    {
        /* Regular driver control */
        // All other states ingore driver input
        DRIVER_CONTROLLING,

        /* Climbing */
        // TODO: We could add align to climb and mount hab platform states, if the drivers want auto align for climbing
        LIFTING_TO_THREE,
        RUNNING_INTAKE_UNTIL_PLATFORM_CONTACT,
        RETRACTING_FORWARD_STRUTS,
        DRIVING_UNTIL_PLATFORM_FULL_SUPPORT,
        RETRACTING_REAR_STRUTS,
        // TODO: Should we have a "drive fully onto the platform"?

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
    private static final double kCargoChuteMovingWaitDuration = 0.8; // Seconds TODO: Tune me
    private static final double kDriveUntilPlatformContactDuration = 1; // Seconds TODO: Tune me
    private static final double kDriveUntilPlatformFullSupportDuration = 1; // Seconds TODO: Tune me

    private static final DriveSignal kPlatformDriveSpeed = new DriveSignal(1, 1);

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
                mStateChangedTimer.reset();
                mStateChangedTimer.start();
                mSystemState = SystemState.DRIVER_CONTROLLING;
                mStateChanged = true;
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
                    // TODO: We have to call mSubsystem.setWantedState in basically all of these branches

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
                        mDrive.setOpenLoop(kPlatformDriveSpeed);
                        if (mStateChangedTimer.hasPeriodPassed(kDriveUntilPlatformContactDuration) && newState == mSystemState)
                            newState = SystemState.RETRACTING_FORWARD_STRUTS;
                        break;
                    case RETRACTING_FORWARD_STRUTS:
                        mClimber.setWantedState(Climber.WantedState.RETRACT_FRONT_STRUTS);
                        if (mClimber.atTarget() && newState == mSystemState)
                            newState = SystemState.DRIVING_UNTIL_PLATFORM_FULL_SUPPORT;
                        break;
                    case DRIVING_UNTIL_PLATFORM_FULL_SUPPORT:
                        mDrive.setOpenLoop(kPlatformDriveSpeed);
                        if (mStateChangedTimer.hasPeriodPassed(kDriveUntilPlatformFullSupportDuration) && newState == mSystemState)
                            newState = SystemState.RETRACTING_REAR_STRUTS;
                        break;
                    case RETRACTING_REAR_STRUTS:
                        mClimber.setWantedState(Climber.WantedState.RETRACT_REAR_STRUTS);
                        if (mWantedState == WantedState.CLIMB && mClimber.atTarget())
                        {
                            mWantedState = WantedState.DRIVER_CONTROL;
                            newState = SystemState.DRIVER_CONTROLLING;
                        }
                        break;

                    /* Placing/intaking game pieces */
                    case INTAKING_AND_ALIGNING_CLOSEST_FORWARD_TARGET:
                        mCargoIntake.setWantedState(CargoIntake.WantedState.INTAKE);
                        mCargoChute.setWantedState(CargoChute.WantedState.BRING_BALL_TO_TOP);
                        if (mStateChanged)
                        {
                            ArrayList<Pose2d> waypoints = new ArrayList<>();
                            waypoints.add(mRobotStateMap.getFieldToVehicle(Timer.getFPGATimestamp()));
                            waypoints.add(Pose2d.identity());

                            double startTime = Timer.getFPGATimestamp();
                            TrajectoryIterator<TimedState<Pose2dWithCurvature>> t =
                                    new TrajectoryIterator<>((new TimedView<>((mTrajectoryGenerator.generateTrajectory(false, waypoints)))));
                            // TODO: Maybe plug in our current velocity as the start veloicty of the path?
                            Logger.info("Path generated; took " + (Timer.getFPGATimestamp() - startTime) + " seconds.");
                            mDrive.setTrajectory(t);
                        }

                        if (mWantedState == WantedState.ALIGN_AND_INTAKE_CARGO && (mDrive.isDoneWithTrajectory() || mCargoChute.atTarget()))
                        {
                            mCargoIntake.setWantedState(CargoIntake.WantedState.HOLD);
                            mDrive.setOpenLoop(DriveSignal.BRAKE);
                            mWantedState = WantedState.DRIVER_CONTROL;
                            newState = SystemState.DRIVER_CONTROLLING;
                        }
                        break;
                    case ALIGNING_CLOSEST_REVERSE_TARGET:
                        // TODO: Put in paths
                        if (mWantedState == WantedState.ALIGN_AND_EJECT_CARGO && mDrive.isDoneWithTrajectory())
                        {
                            newState = SystemState.EJECTING_CARGO;
                        }
                        else if (mWantedState == WantedState.ALIGN_AND_EJECT_PANEL)
                        {
                            mCargoIntake.setWantedState(CargoIntake.WantedState.HOLD);
                            if (mDrive.isDoneWithTrajectory())
                                newState = SystemState.EJECTING_PANEL;
                        }
                        break;
                    case INTAKING_PANEL:
                        // This is literally just a timer and setting the cargo chute to low
                        // TODO: Add cargo chute bring to low wanted state
                        if (mWantedState == WantedState.ALIGN_AND_INTAKE_PANEL
                                && mStateChangedTimer.hasPeriodPassed(kPanelHandlingDuration))
                        {
                            mWantedState = WantedState.DRIVER_CONTROL;
                            newState = SystemState.DRIVER_CONTROLLING;
                        }
                        break;
                    case MOVING_CHUTE_TO_EJECT_PANEL:
                        // TODO: Bring cargo chute to low here
                        if (newState == mSystemState && mStateChangedTimer.hasPeriodPassed(kCargoChuteMovingWaitDuration))
                            newState = SystemState.EJECTING_PANEL;
                    case EJECTING_PANEL:
                        // TODO: Also bring cargo chute to low here
                        if (mCargoChute.atTarget())
                            mPanelHandler.setWantedState(PanelHandler.WantedState.EJECT);

                        if (newState == mSystemState && mStateChangedTimer.hasPeriodPassed(kPanelHandlingDuration)
                                && mCargoChute.atTarget() && mPanelHandler.atTarget())
                            newState = SystemState.BACKING_OUT_FROM_LOADING;
                        break;
                    case EJECTING_CARGO:
                        // TODO: There are multiple eject levels, so we either let the drivers press
                        // eject themselves or have multiple wanted states for each eject level
                        if (mWantedState == WantedState.ALIGN_AND_EJECT_CARGO)
                        {
                            mWantedState = WantedState.DRIVER_CONTROL;
                            newState = SystemState.DRIVER_CONTROLLING;
                        }
                        break;
                    case BACKING_OUT_FROM_LOADING:
                        // TODO: Put in paths
                        if (mWantedState == WantedState.ALIGN_AND_EJECT_CARGO && mDrive.isDoneWithTrajectory() && newState == mSystemState)
                            newState = SystemState.TURNING_AROUND;
                        break;
                    case TURNING_AROUND:
                        // TODO: Implement drive quick turn
                        if (mWantedState == WantedState.ALIGN_AND_EJECT_PANEL)
                        {
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
        }
    };

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
                        mSystemState == SystemState.INTAKING_PANEL)
                    break;
                newState = SystemState.ALIGNING_CLOSEST_REVERSE_TARGET;
                break;
            case ALIGN_AND_EJECT_CARGO:
                if (mSystemState == SystemState.INTAKING_AND_ALIGNING_CLOSEST_FORWARD_TARGET ||
                        mSystemState == SystemState.EJECTING_CARGO)
                    break;
                newState = SystemState.ALIGNING_CLOSEST_REVERSE_TARGET;
                break;
            case ALIGN_AND_EJECT_PANEL:
                if (mSystemState == SystemState.ALIGNING_CLOSEST_REVERSE_TARGET ||
                        mSystemState == SystemState.EJECTING_PANEL ||
                        mSystemState == SystemState.BACKING_OUT_FROM_LOADING ||
                        mSystemState == SystemState.TURNING_AROUND)
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
            case EJECT_PANEL:
                if (mSystemState == SystemState.MOVING_CHUTE_TO_EJECT_PANEL
                        || mSystemState == SystemState.EJECTING_PANEL)
                    newState = SystemState.MOVING_CHUTE_TO_EJECT_PANEL;
            default:
                logError("Unhandled wanted state in default state transfer!");
                newState = SystemState.DRIVER_CONTROLLING;
                break;
        }
        return newState;
    }

    public synchronized void setWantedState(WantedState wantedState)
    {
        logNotice("Wanted state to " + wantedState.toString());
        mWantedState = wantedState;
    }

    public synchronized void reverseDrivingDirection()
    {
        mIsReversed = !mIsReversed;
    }

    public synchronized boolean isDrivingReversed()
    {
        return mIsReversed;
    }

    public synchronized boolean isDriverControlled()
    {
        return mSystemState == SystemState.DRIVER_CONTROLLING;
    }

    @Override
    public void stop()
    {
        // Subsystem manager stops these, we don't
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
    }
}
