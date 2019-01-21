package com.spartronics4915.frc2019.subsystems;

import javax.lang.model.util.ElementScanner6;

import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;

import edu.wpi.first.wpilibj.Solenoid;

public class Climber extends Subsystem
{ //no climb until last 30 sec, 2 DISTANCE SENSORS on diff sides, back-up: check if motor isn't running,
 //double solenoids, 2 seperate solenoids(4 total), auto & slow (superstructure), 16 in gap, tanks,
// disabling the robot - solenoid on/off,

    private static Climber mInstance = null;

    public static Climber getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new Climber();
        }
        return mInstance;
    }

    public enum WantedState
    {
        RETRACTED, FRONTCLIMB, BACKCLIMB,
    }

    private enum SystemState
    {
        RETRACTING, FRONTCLIMBING, BACKCLIMBING,
    }

    private WantedState mWantedState = WantedState.RETRACTED;
    private SystemState mSystemState = SystemState.RETRACTING;

    private static final boolean kSolenoidLift = true;
    private static final boolean kSolenoidRetract = false;

    private Solenoid mSolenoid1 = null;
    private Solenoid mSolenoid2 = null;
    private Solenoid mSolenoid3 = null;
    private Solenoid mSolenoid4 = null;

    private Climber()
    {
        boolean success = true;
        try
        {
            //mSolenoid1 = new Solenoid(1);
        }
        catch (Exception e)
        {
            success = false;
            logException("Couldn't instantiate hardware", e);
        }

        logInitialized(success);
    }

    private final ILoop mLoop = new ILoop()
    {

        @Override
        public void onStart(double timestamp)
        {
            synchronized (Climber.this)
            {
                mWantedState = WantedState.RETRACTED;
                mSystemState = SystemState.RETRACTING;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (Climber.this)
            {
                outputTelemetry();
                SystemState newState = defaultStateTransfer();
                switch (mSystemState)
                {
                    case FRONTCLIMBING:
                        mSolenoid1.set(kSolenoidLift);
                        mSolenoid2.set(kSolenoidLift);
                        mSolenoid3.set(kSolenoidRetract);
                        mSolenoid4.set(kSolenoidRetract);
                        break;
                    case BACKCLIMBING:
                        mSolenoid1.set(kSolenoidRetract);
                        mSolenoid2.set(kSolenoidRetract);
                        mSolenoid3.set(kSolenoidLift);
                        mSolenoid4.set(kSolenoidLift);
                        break;
                    case RETRACTING:
                        stop();
                        break;
                    default:
                        logError("Unhandled system state!");
                }
                mSystemState = newState;
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            synchronized (Climber.this)
            {
                stop();
            }
        }
    };

    private SystemState defaultStateTransfer()
    {
        SystemState newState = mSystemState;
        switch (mWantedState)
        {
            case RETRACTED:
                if(mWantedState == WantedState.FRONTCLIMB)
                    newState = SystemState.FRONTCLIMBING;
                else
                    newState = SystemState.RETRACTING;
                break;
            case FRONTCLIMB:
                if(mWantedState == WantedState.BACKCLIMB)
                    newState = SystemState.BACKCLIMBING;
                else
                    newState = SystemState.FRONTCLIMBING;
                break;
            case BACKCLIMB:
                if(mWantedState == WantedState.RETRACTED)
                    newState = SystemState.RETRACTING;
                else
                    newState = SystemState.BACKCLIMBING;
                break;
            default:
                newState = SystemState.RETRACTING;
                break;
        }
        return newState;
    }
    /*What we need each state to do:
    Retract: double solenoids retracted, used when driving or staying on a platform, if disabled 
    Frontclimb: Front double solenoids extended, make sure happens only when distance sensors are in range & within area of HAB,
    motors are not running, in last 30 sec, */

    public synchronized void setWantedState(WantedState wantedState)
    {
        mWantedState = wantedState;
    }

    public synchronized boolean atTarget()
    {
        return true;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper)
    {
        enabledLooper.register(mLoop);
    }

    @Override
    public boolean checkSystem(String variant)
    {
        return false;
    }

    @Override
    public void outputTelemetry()
    {
        dashboardPutState(mSystemState.toString());
        dashboardPutWantedState(mWantedState.toString());
    }

    @Override
    public void stop()
    {
        mSolenoid1.set(kSolenoidRetract);
        mSolenoid2.set(kSolenoidRetract);
        mSolenoid3.set(kSolenoidRetract);
        mSolenoid4.set(kSolenoidRetract);
    }
}
