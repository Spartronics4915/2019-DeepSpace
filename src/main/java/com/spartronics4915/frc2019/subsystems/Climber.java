package com.spartronics4915.frc2019.subsystems;

import javax.lang.model.util.ElementScanner6;

import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;

import edu.wpi.first.wpilibj.Solenoid;

public class Climber extends Subsystem
{

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
        RETRACTED, FRONTCLIMB, BACKCLIMB,// FRONT2CLIMB, BACK2CLIMB,
    }

    private enum SystemState
    {
        RETRACTING, FRONTCLIMBING, BACKCLIMBING,// FRONT2CLIMBING, BACK2CLIMBING,
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
                    /*case FRONT2CLIMBING:
                        break;
                    case BACK2CLIMBING:
                        break;*/
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
                //if(mWantedState == WantedState.FRONT2CLIMB)
                //    newState = SystemState.FRONT2CLIMBING;
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
            /*case FRONT2CLIMB:
                if(mWantedState == WantedState.BACK2CLIMB)
                    newState = SystemState.BACK2CLIMBING;
                else
                    newState = SystemState.FRONT2CLIMBING;
                break;
            case BACK2CLIMB:
                if(mWantedState == WantedState.RETRACTED)
                    newState = SystemState.RETRACTING;
                else
                    newState = SystemState.BACKCLIMBING;
                break;*/
            default:
                newState = SystemState.RETRACTING;
                break;
        }
        return newState;
    }

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
