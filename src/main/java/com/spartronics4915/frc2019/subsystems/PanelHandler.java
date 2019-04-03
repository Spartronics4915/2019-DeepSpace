package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.lib.util.CANProbe;
import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.DigitalInput;


/** 2 pneumatics are used for pushing the hatch out
 * 1 pneumatic is used for controlling the arm which is
 * used to hold the hatch in place, for a total of 3 pneumatics
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
        ARM_DOWN, ARM_UP, EJECT_ROCKET
    }

    private enum SystemState
    {
        ARM_DOWNING, ARM_UPING, EJECTING_ROCKET
    }

    private WantedState mWantedState = WantedState.ARM_UP;
    private SystemState mSystemState = SystemState.ARM_UPING;

    private Solenoid mSlideSolenoid = null;
    private Solenoid mArmSolenoid = null;
    private Timer mStateChangedTimer = new Timer();
    //private DigitalInput mLimitSwitch = null;

    private boolean mStateChanged;

    private PanelHandler()
    {
        boolean success = false;
        try
        {
            if (!CANProbe.getInstance().validatePCMId(Constants.kCargoHatchArmPCMId)) throw new RuntimeException("PanelHandler PCM isn't on the CAN bus!");

            mSlideSolenoid = new Solenoid(Constants.kCargoHatchArmPCMId, Constants.kPanelHandlerSlideSolenoidId);
            mArmSolenoid = new Solenoid(Constants.kCargoHatchArmPCMId, Constants.kPanelHandlerArmSolenoidId); // TODO: Change this
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
        @Override
        public void onStart(double timestamp)
        {
            synchronized (PanelHandler.this)
            {
                mSlideSolenoid.set(Constants.kPanelSlideSolenoidRetract);
                mArmSolenoid.set(Constants.kPanelArmSolenoidDown);
                mStateChangedTimer.reset();
                mStateChangedTimer.start();
                mStateChanged = true;
                mWantedState = WantedState.ARM_UP;
                mSystemState = SystemState.ARM_UPING;
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
                    case ARM_DOWNING:
                        if (mStateChanged)
                            mArmSolenoid.set(Constants.kPanelArmSolenoidDown);
                        break;
                    case ARM_UPING:
                        if (mStateChanged)
                            mArmSolenoid.set(Constants.kPanelArmSolenoidUp);
                        break;
                    case EJECTING_ROCKET:
                        if (mStateChanged)
                            mArmSolenoid.set(Constants.kPanelArmSolenoidUp);
                        else if (mStateChangedTimer.hasPeriodPassed(Constants.kPanelSlideExtendTime))
                            mSlideSolenoid.set(Constants.kPanelSlideSolenoidRetract);
                        else if (mStateChangedTimer.hasPeriodPassed(Constants.kPanelArmMovementTime))
                            mSlideSolenoid.set(Constants.kPanelSlideSolenoidExtend);
                        break;
                    default:
                        logError("Unhandled system state!");
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
            case ARM_DOWN:
                newState = SystemState.ARM_DOWNING;
                break;
            case ARM_UP:
                newState = SystemState.ARM_UPING;
                break;
            case EJECT_ROCKET:
                newState = SystemState.EJECTING_ROCKET;
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
            case ARM_DOWN:
                return mSystemState == SystemState.ARM_DOWNING && mStateChangedTimer.hasPeriodPassed(Constants.kPanelArmMovementTime);
            case ARM_UP:
                return mSystemState == SystemState.ARM_UPING && mStateChangedTimer.hasPeriodPassed(Constants.kPanelArmMovementTime);
            case EJECT_ROCKET:
                return false;
            default:
                logError("atTarget for unknown wanted state " + mWantedState);
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
        logNotice("Starting PanelHandler Solenoid Check");
        try
        {
            logNotice("Extending SlideSolenoid for 2 seconds");
            mSlideSolenoid.set(Constants.kPanelSlideSolenoidExtend);
            Timer.delay(2);
            logNotice("Retracting SlideSolenoid for 2 seconds");
            mSlideSolenoid.set(Constants.kPanelSlideSolenoidRetract);
            Timer.delay(2);
            logNotice("Extending ArmSolenoid for 2 seconds");
            mArmSolenoid.set(Constants.kPanelArmSolenoidUp);
            Timer.delay(2);
            logNotice("Retracting ArmSolenoid for 2 seconds");
            mArmSolenoid.set(Constants.kPanelArmSolenoidDown);
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
        dashboardPutBoolean("mSlideSolenoid Extended", mSlideSolenoid.get());
        dashboardPutBoolean("mArmSolenoid Extended", mArmSolenoid.get());
    }

    @Override
    public void stop()
    {
        mSlideSolenoid.set(Constants.kPanelSlideSolenoidRetract);
        mArmSolenoid.set(Constants.kPanelArmSolenoidUp);
    }
}