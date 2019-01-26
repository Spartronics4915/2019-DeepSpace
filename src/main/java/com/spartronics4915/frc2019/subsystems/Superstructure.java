package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.lib.util.ILooper;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.RobotStateMap;

import java.util.ArrayList;

import com.spartronics4915.frc2019.paths.TrajectoryGenerator;
import com.spartronics4915.frc2019.planners.DriveMotionPlanner;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.geometry.Rotation2d;
import com.spartronics4915.lib.trajectory.TimedView;
import com.spartronics4915.lib.trajectory.Trajectory;
import com.spartronics4915.lib.trajectory.TrajectoryIterator;
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
        // TODO: Can we do a climb to level two?
        CLIMB_TO_THREE,
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
        DRIVING_UNTIL_PLATFORM_CONTACT, // TODO: Is this going to be a mechanism that deploys and pulls us forward ("the cheetos")?
        RETRACTING_FORWARD_STRUTS,
        DRIVING_UNTIL_PLATFORM_FULL_SUPPORT,
        RETRACTING_REAR_STRUTS,
        // TODO: Should we have a "drive fully onto the platform"?

        /* Placing/intaking game pieces */
        // Alignment using vision+odometry (step 1)
        // TODO: Ensure that forward only has cargo intake on it, and that reverse has cargo out+hatch panel (this is currently assumed below)
        ALIGNING_CLOSEST_FORWARD_TARGET,
        ALIGNING_CLOSEST_REVERSE_TARGET,
        // Intaking (step 2)
        INTAKING_CARGO,
        INTAKING_PANEL,
        // Placing (could also be step 2)
        EJECTING_PANEL,
        MOVING_CARGO_EJECTOR, // There are two eject heights (step 2.1)
        EJECTING_CARGO, // (step 2.2)
        // Backing out and turning (step 3, PANEL panels only)
        BACKING_OUT_FROM_LOADING,
        TURNING_AROUND,
    }

    // Superstructure doesn't own the drive, but needs to access it
    private final Drive mDrive = Drive.getInstance();
    private final CargoHandler mCargoHandler = CargoHandler.getInstance();
    private final Climber mClimber = Climber.getInstance();
    private final PanelHandler mPanelHandler = PanelHandler.getInstance();

    private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private RobotStateMap mRobotStateEstimator = RobotStateEstimator.getInstance().getEncoderRobotStateMap();

    private final double kPanelHandlingDuration = 0.3; // Seconds TODO: Tune me (also is this our responsibility?)
    private final double kDriveUntilPlatformContactDuration = 1; // Seconds TODO: Tune me
    private final double kDriveUntilPlatformFullSupportDuration = 1; // Seconds TODO: Tune me

    private final DriveSignal kPlatformDriveSpeed = new DriveSignal(1, 1);

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

        // State change timestamps are currently unused, but I'm keeping them
        // here because they're potentially useful.
        private double mCurrentStateStartTime;
        private boolean mStateChanged;

        @Override
        public void onStart(double timestamp)
        {
            synchronized (Superstructure.this)
            {
                mWantedState = WantedState.DRIVER_CONTROL;
                mCurrentStateStartTime = timestamp;
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
                        // XXX: Climb sequence is currently uninterruptable (that's why there's no newState == mSystemState)
                        if (mClimber.atTarget())
                            newState = SystemState.DRIVING_UNTIL_PLATFORM_CONTACT;
                        break;
                    case DRIVING_UNTIL_PLATFORM_CONTACT:
                        mDrive.setOpenLoop(kPlatformDriveSpeed);
                        if (Timer.getFPGATimestamp() - mCurrentStateStartTime > kDriveUntilPlatformContactDuration)
                            newState = SystemState.RETRACTING_FORWARD_STRUTS;
                        break;
                    case RETRACTING_FORWARD_STRUTS:
                        if (mClimber.atTarget())
                            newState = SystemState.DRIVING_UNTIL_PLATFORM_FULL_SUPPORT;
                        break;
                    case DRIVING_UNTIL_PLATFORM_FULL_SUPPORT:
                        mDrive.setOpenLoop(kPlatformDriveSpeed);
                        if (Timer.getFPGATimestamp() - mCurrentStateStartTime > kDriveUntilPlatformFullSupportDuration)
                            newState = SystemState.RETRACTING_REAR_STRUTS;
                        break;
                    case RETRACTING_REAR_STRUTS:
                        if (mWantedState == WantedState.CLIMB_TO_THREE && mClimber.atTarget())
                        {
                            mWantedState = WantedState.DRIVER_CONTROL;
                            newState = SystemState.DRIVER_CONTROLLING;
                        }
                        break;

                    /* Placing/intaking game pieces */
                    case ALIGNING_CLOSEST_FORWARD_TARGET:
                        // TODO: Put in paths
                        if (newState == mSystemState)
                        {
                            ArrayList<Pose2d> waypoints = new ArrayList<>();
                            waypoints.add(mRobotStateEstimator.getFieldToVehicle(Timer.getFPGATimestamp()));
                            waypoints.add(Pose2d.identity());

                            double startTime = Timer.getFPGATimestamp();
                            TrajectoryIterator<TimedState<Pose2dWithCurvature>> t =
                                    new TrajectoryIterator<>((new TimedView<>((mTrajectoryGenerator.generateTrajectory(false, waypoints)))));
                            Logger.debug("Path generated; took " + (Timer.getFPGATimestamp() - startTime) + " seconds.");
                            mDrive.setTrajectory(t);
                        }

                        if (newState == mSystemState && mDrive.isDoneWithTrajectory())
                            newState = SystemState.INTAKING_CARGO;
                        break;
                    case ALIGNING_CLOSEST_REVERSE_TARGET:
                        // TODO: Put in paths
                        if (mWantedState == WantedState.ALIGN_AND_EJECT_CARGO)
                            newState = SystemState.MOVING_CARGO_EJECTOR;
                        else if (mWantedState == WantedState.ALIGN_AND_EJECT_PANEL)
                            newState = SystemState.EJECTING_PANEL;
                        break;
                    case INTAKING_CARGO:
                        // TODO: Should this be isCargoHeld() or a timer?
                        if (mWantedState == WantedState.ALIGN_AND_INTAKE_CARGO)
                        {
                            mWantedState = WantedState.DRIVER_CONTROL; // XXX: Bad?
                            newState = SystemState.DRIVER_CONTROLLING;
                        }
                        break;
                    case INTAKING_PANEL:
                        if (mWantedState == WantedState.ALIGN_AND_INTAKE_PANEL
                                && Timer.getFPGATimestamp() - mCurrentStateStartTime > kPanelHandlingDuration)
                        {
                            mWantedState = WantedState.DRIVER_CONTROL;
                            newState = SystemState.DRIVER_CONTROLLING;
                        }
                        break;
                    case EJECTING_PANEL:
                        if (newState == mSystemState && Timer.getFPGATimestamp() - mCurrentStateStartTime > kPanelHandlingDuration)
                            newState = SystemState.BACKING_OUT_FROM_LOADING;
                        break;
                    case MOVING_CARGO_EJECTOR:
                        if (newState == mSystemState && mCargoHandler.atTarget())
                            newState = SystemState.EJECTING_CARGO;
                        break;
                    case EJECTING_CARGO:
                        // TODO: Should this be isCargoEjected(), atTarget(), or a timer?
                        if (mWantedState == WantedState.ALIGN_AND_EJECT_CARGO)
                        {
                            mWantedState = WantedState.DRIVER_CONTROL;
                            newState = SystemState.DRIVER_CONTROLLING;
                        }
                        break;
                    case BACKING_OUT_FROM_LOADING:
                        // TODO: Put in paths
                        if (newState == mSystemState && mDrive.isDoneWithTrajectory())
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
                    mStateChanged = true;
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
                if (mSystemState == SystemState.ALIGNING_CLOSEST_FORWARD_TARGET ||
                        mSystemState == SystemState.INTAKING_CARGO)
                    break;
                newState = SystemState.ALIGNING_CLOSEST_FORWARD_TARGET;
                break;
            case ALIGN_AND_INTAKE_PANEL:
                // Assumes PANEL intake is on the reverse side
                if (mSystemState == SystemState.ALIGNING_CLOSEST_REVERSE_TARGET ||
                        mSystemState == SystemState.INTAKING_PANEL)
                    break;
                newState = SystemState.ALIGNING_CLOSEST_REVERSE_TARGET;
                break;
            case ALIGN_AND_EJECT_CARGO:
                if (mSystemState == SystemState.ALIGNING_CLOSEST_FORWARD_TARGET ||
                        mSystemState == SystemState.MOVING_CARGO_EJECTOR ||
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
            case CLIMB_TO_THREE:
                if (mSystemState == SystemState.LIFTING_TO_THREE ||
                        mSystemState == SystemState.DRIVING_UNTIL_PLATFORM_CONTACT ||
                        mSystemState == SystemState.RETRACTING_FORWARD_STRUTS ||
                        mSystemState == SystemState.DRIVING_UNTIL_PLATFORM_FULL_SUPPORT ||
                        mSystemState == SystemState.RETRACTING_REAR_STRUTS)
                    break;
                newState = SystemState.LIFTING_TO_THREE;
                break;
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
        // TODO: Figure out all the "stopped" states of the subsystems and set them here
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
