package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.lib.util.CANProbe;
import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.DigitalInput;

/**
 * BACK
 * OPEN
 * CLOSE
 */

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
        INSIDE_FRAME, OPEN, CLOSE
    }

    private enum SystemState
    {
        INSIDE_FRAMING, OPENING, CLOSING
    }

    private WantedState mWantedState = WantedState.INSIDE_FRAME;
    private SystemState mSystemState = SystemState.INSIDE_FRAMING;

    private Solenoid mArmSolenoid = null;
    private Solenoid mSlideSolenoid = null;
    private Timer mPanelTimer = new Timer();

    private boolean mStateChanged;

    private PanelHandler()
    {
        boolean success = false;
        try
        {
            if (!CANProbe.getInstance().validatePCMId(Constants.kCargoHatchArmPCMId)) throw new RuntimeException("PanelHandler PCM isn't on the CAN bus!");

            mArmSolenoid = new Solenoid(Constants.kCargoHatchArmPCMId, Constants.kPanelHandlerSolenoid); // panel pneumatic goes to hatch arm
            mSlideSolenoid = new Solenoid(Constants.kCargoHatchArmPCMId, Constants.kRampSolenoidId); // chute pneumatic goes to arm
            success = true;
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
                mArmSolenoid.set(Constants.kPanelSolenoidRetract);
                mSlideSolenoid.set(Constants.kPanelSolenoidRetract);
                mStateChanged = true;
                mWantedState = WantedState.INSIDE_FRAME;
                mSystemState = SystemState.INSIDE_FRAMING;
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
                    case INSIDE_FRAMING:
                        if (mStateChanged)
                        {
                            mArmSolenoid.set(Constants.kPanelSolenoidRetract);
                            mSlideSolenoid.set(Constants.kPanelSolenoidRetract);
                        }
                        break;
                    case OPENING://BB6
                        if (mStateChanged)
                        {
                            mArmSolenoid.set(Constants.kPanelSolenoidExtend);
                            mSlideSolenoid.set(Constants.kPanelSolenoidExtend);
                        }
                        break;
                    case CLOSING:
                        if (mStateChanged)
                        {
                            mArmSolenoid.set(Constants.kPanelSolenoidRetract);
                            mPanelTimer.delay(1);
                            mSlideSolenoid.set(Constants.kPanelSolenoidRetract);
                        }
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
            case INSIDE_FRAME:
                newState = SystemState.INSIDE_FRAMING;
                break;
            case OPEN:
                newState = SystemState.OPENING;
                break;
            case CLOSE:
                newState = SystemState.CLOSING;
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
        return mSystemState == SystemState.INSIDE_FRAMING && mWantedState == WantedState.INSIDE_FRAME;
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
            logNotice("Extending ArmSolenoid for 2 seconds");
            mArmSolenoid.set(Constants.kPanelSolenoidExtend);
            Timer.delay(2);
            logNotice("Retracting ArmSolenoid for 2 seconds");
            mArmSolenoid.set(Constants.kPanelSolenoidRetract);

            logNotice("Extending SlideSolenoid for 2 seconds");
            mSlideSolenoid.set(Constants.kPanelSolenoidExtend);
            Timer.delay(2);
            logNotice("Retracting SlideSolenoid for 2 seconds");
            mSlideSolenoid.set(Constants.kPanelSolenoidRetract);
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
        dashboardPutBoolean("mArmSolenoid Extended", mArmSolenoid.get());
        dashboardPutBoolean("mSlideSolenoid Extended", mSlideSolenoid.get());
    }

    @Override
    public void stop()
    {
        mArmSolenoid.set(Constants.kPanelSolenoidExtend);
        mSlideSolenoid.set(Constants.kPanelSolenoidExtend);
    }
}