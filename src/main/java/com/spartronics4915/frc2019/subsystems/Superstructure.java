package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.frc2019.loops.ILooper;
import com.spartronics4915.frc2019.loops.Loop;
import com.spartronics4915.frc2019.loops.Looper;

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

    // Superstructure doesn't own the drive, but needs to access it
    private final Drive mDrive = Drive.getInstance();

    // Internal state of the system
    public enum SystemState
    {
        IDLE,
    };

    // Desired function from user
    public enum WantedState
    {
        IDLE,
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    // State change timestamps are currently unused, but I'm keeping them
    // here because they're potentially useful.
    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    private Timer mTimer = new Timer();

    private Superstructure()
    {
        logInitialized(true);
    }

    private Loop mLoop = new Loop()
    {

        // Every time we transition states, we update the current state start
        // time and the state changed boolean (for one cycle)
        private double mWantStateChangeStartTime;

        @Override
        public void onStart(double timestamp)
        {
            synchronized (Superstructure.this)
            {
                mWantedState = WantedState.IDLE;
                mCurrentStateStartTime = timestamp;
                mWantStateChangeStartTime = timestamp;
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (Superstructure.this)
            {
                SystemState newState = mSystemState;
                switch (mSystemState)
                {
                    case IDLE:
                        switch (mWantedState)
                        {
                            default: // either idle or unimplemented
                                break;
                        }
                        break;
                    default:
                        newState = defaultStateTransfer();
                }
                if (mSystemState != newState)
                {
                    if (newState == SystemState.IDLE)
                    {
                        // need to reset subsystems to an idle?
                    }
                }
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
            case IDLE:
                newState = SystemState.IDLE;
                break;
            default:
                newState = SystemState.IDLE;
                break;
        }
        return newState;
    }

    public synchronized void setWantedState(WantedState wantedState)
    {
        logNotice("Wanted state to " + wantedState.toString());
        mWantedState = wantedState;
    }

    /**
     * This is a bit like the atTarget methods of many other subsystem. It's called
     * something different because Superstructure is fundamentally different from
     * other subsystems.
     */
    public synchronized boolean isIdling()
    {
        return mSystemState == SystemState.IDLE;
    }

    @Override
    public void stop()
    {

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
    public boolean checkSystem()
    {
        logNotice("checkSystem not implemented");
        return false;
    }

    @Override
    public void outputTelemetry()
    {

    }
}
