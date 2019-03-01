package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.lib.util.CANProbe;
import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.DigitalInput;


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
    //private DigitalInput mLimitSwitch = null;

    private boolean mStateChanged;

    private PanelHandler()
    {
        boolean success = false;
        try
        {
            if (!CANProbe.getInstance().validatePCMId(Constants.kCargoHatchArmPCMId)) throw new RuntimeException("PanelHandler PCM isn't on the CAN bus!");

            mSlideSolenoid = new Solenoid(Constants.kCargoHatchArmPCMId, Constants.kPanelHandlerSlideSolenoid);
            mArmSolenoid = new Solenoid(Constants.kCargoHatchArmPCMId, Constants.kPanelHandlerArmSolenoid); // TODO: Change this
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
        private double mArmDownTime;
        private double mRetractTime;

        @Override
        public void onStart(double timestamp)
        {
            synchronized (PanelHandler.this)
            {
                mSlideSolenoid.set(Constants.kPanelSolenoidRetract);
                mArmSolenoid.set(Constants.kPanelSolenoidRetract);
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
                        {
                            mArmSolenoid.set(Constants.kPanelSolenoidRetract);
                        }
                    case ARM_UPING:
                        if (mStateChanged)
                        {
                            mArmSolenoid.set(Constants.kPanelSolenoidExtend);
                        }
                    case EJECTING_ROCKET:
                        if (mStateChanged)
                        {
                            mArmSolenoid.set(Constants.kPanelSolenoidRetract);
                            mArmDownTime = Timer.getFPGATimestamp();
                        }
                        else if (Timer.getFPGATimestamp() > mArmDownTime + Constants.kPanelArmDownTime){
                            mSlideSolenoid.set(Constants.kPanelSolenoidExtend);
                            mEjectTime = Timer.getFPGATimestamp();
                        } 
                        else if (Timer.getFPGATimestamp() > mEjectTime + Constants.kPanelEjectTime){
                            mSlideSolenoid.set(Constants.kPanelSolenoidRetract);
                            mRetractTime = Timer.getFPGATimestamp();
                        }
                        else if (Timer.getFPGATimestamp() > mRetractTime + Constants.kPanelRetractTime && newState == mSystemState && mWantedState == WantedState.EJECT_ROCKET){
                            setWantedState(WantedState.ARM_UP);
                        }
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
                return mSystemState == SystemState.ARM_DOWNING;
            case ARM_UP:
                return mSystemState == SystemState.ARM_UPING;
            case EJECT_ROCKET:
                return mSystemState == SystemState.EJECTING_ROCKET;
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
            mSlideSolenoid.set(Constants.kPanelSolenoidExtend);
            Timer.delay(2);
            logNotice("Retracting SlideSolenoid for 2 seconds");
            mSlideSolenoid.set(Constants.kPanelSolenoidRetract);
            Timer.delay(2);
            logNotice("Extending ArmSolenoid for 2 seconds");
            mArmSolenoid.set(Constants.kPanelSolenoidExtend);
            Timer.delay(2);
            logNotice("Retracting ArmSolenoid for 2 seconds");
            mArmSolenoid.set(Constants.kPanelSolenoidRetract);
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
        mSlideSolenoid.set(Constants.kPanelSolenoidRetract);
        mArmSolenoid.set(Constants.kPanelSolenoidExtend);
    }
}