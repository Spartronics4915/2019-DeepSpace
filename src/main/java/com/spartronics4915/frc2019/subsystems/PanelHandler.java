package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.frc2019.Constants;
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

    private Solenoid mSolenoid = null;

    private boolean mStateChanged;

    private PanelHandler()
    {
        boolean success = true;
        try
        {
            mSolenoid = new Solenoid(Constants.kPanelHandlerSolenoid);
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
        private double mEjectTime;

        @Override
        public void onStart(double timestamp)
        {
            synchronized (PanelHandler.this)
            {
                mSolenoid.set(kSolenoidRetract);
                mWantedState = WantedState.RETRACT;
                mSystemState = SystemState.RETRACTING;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (PanelHandler.this)
            {
                SystemState newState = defaultStateTransfer();
                switch (mSystemState)
                {
                    case RETRACTING:
                        if (mStateChanged)
                        {
                            mSolenoid.set(kSolenoidRetract);
                        }
                        break;
                    case EJECTING://BB6
                        if (mStateChanged)
                        {
                            mSolenoid.set(kSolenoidExtend);
                            mEjectTime = Timer.getFPGATimestamp();
                        }
                        else if (Timer.getFPGATimestamp() > mEjectTime + kEjectTime && newState == mSystemState)
                            setWantedState(WantedState.RETRACT);
                        break;
                    default:
                        logError("Unhandled system state!");
                }
                if (newState != mSystemState)
                {
                    mStateChanged = true;
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
        return mSystemState == SystemState.RETRACTING && mWantedState == WantedState.RETRACT;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper)
    {
        enabledLooper.register(mLoop);
    }

    @Override
    public boolean checkSystem(String variant)//DS6
    {
        logNotice("Starting PanelHandler Solenoid Check");
        try
        {
            logNotice("Extending solenoid for 2 seconds");
            mSolenoid.set(kSolenoidExtend);
            Timer.delay(2);
            logNotice("Retracting solenoid for 2 seconds");
            mSolenoid.set(kSolenoidRetract);
        }
        catch (Exception e)
        {
            logException("Trouble instantiating hardware ", e);
            return false;
        }
        logNotice("PanelHandler Solenoid Check End");
        return true;
    }

    @Override
    public void outputTelemetry()
    {
        dashboardPutState(mSystemState.toString());
        dashboardPutWantedState(mWantedState.toString());
        dashboardPutBoolean("mSolenoid1 Extended", mSolenoid.get());
    }

    @Override
    public void stop()
    {
        mSolenoid.set(kSolenoidRetract);
    }
}