package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;


/** 2 pneumatics to eject panels
 * panels held on by velcro */

public class PanelHandler extends Subsystem
{ 
    private static PanelHandler mInstance = null;

    public static PanelHandler getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new PanelHandler();
        }
        return mInstance;
    }

    public enum WantedState
    {
        RETRACT, EJECT
    }

    private enum SystemState
    {
        RETRACTING, EJECTING
    }

    private WantedState mWantedState = WantedState.RETRACT;
    private SystemState mSystemState = SystemState.RETRACTING;

    private final double kEjectTime = 0.3; // Seconds TODO: Tune me
    private static final boolean kSolenoidExtend = true;
    private static final boolean kSolenoidRetract = false;

    private Solenoid mSolenoid1 = null;

    private double time;

    private PanelHandler()
    {
        boolean success = true;
        try
        {
            mSolenoid1 = new Solenoid(4);
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
            synchronized (PanelHandler.this)
            {
                mSolenoid1.set(kSolenoidRetract);
                mWantedState = WantedState.RETRACT;
                mSystemState = SystemState.RETRACTING;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (PanelHandler.this)
            {
                outputTelemetry();
                SystemState newState = defaultStateTransfer();
                switch (mSystemState)
                {
                    case RETRACTING:
                        if (newState != mSystemState)
                        {
                            mSolenoid1.set(kSolenoidRetract);
                        }
                        break;
                    case EJECTING:
                        if (newState != mSystemState)
                        {
                            mSolenoid1.set(kSolenoidExtend);
                            time = Timer.getFPGATimestamp();
                        }
                        else if (Timer.getFPGATimestamp() > time + kEjectTime)
                            setWantedState(WantedState.RETRACT);
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
            synchronized (PanelHandler.this)
            {
                stop();
            }
        }
    };

    private SystemState defaultStateTransfer() //Eject -timer-> Retract
    {
        SystemState newState = mSystemState;
        switch (mWantedState)
        {
            case RETRACT:
                newState = SystemState.RETRACTING;
                break;
            case EJECT:
                newState = SystemState.EJECTING;
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
        switch (mWantedState)
        {
            case RETRACT:
                return mSystemState == SystemState.RETRACTING;
            case EJECT:
                return mSystemState == SystemState.EJECTING;
            default:
                return false;
        }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper)
    {
        enabledLooper.register(mLoop);
    }

    @Override
    public boolean checkSystem(String variant)
    {
        /*Ensure solenoids are functioning
        */
        logNotice("Starting PanelHandler Solenoid Check");
        try
        {
            mSolenoid1.set(kSolenoidExtend);
            Timer.delay(2);
            mSolenoid1.set(kSolenoidRetract);
        }
        catch (Exception e)
        {
            logException("Trouble instantiating hardware ", e);
            return false;
        }
        logNotice("PanelHandler Solenoid Check End - Successful");
        return true;
    }

    @Override
    public void outputTelemetry()
    {
        dashboardPutState(mSystemState.toString());
        dashboardPutWantedState(mWantedState.toString());
        dashboardPutBoolean("mSolenoid1 Extended", mSolenoid1.get());
    }

    @Override
    public void stop()
    {
        mSolenoid1.set(kSolenoidRetract);
    }
}