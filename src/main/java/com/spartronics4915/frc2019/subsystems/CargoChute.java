package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.lib.drivers.TalonSRXFactory;
import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;
import com.spartronics4915.lib.util.Logger;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CargoChute extends Subsystem
{

    private static CargoChute mInstance = null;

    public static CargoChute getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new CargoChute();
        }
        return mInstance;
    }

    public enum WantedState // Each WantedState will correspond to a button
    {
        MANUAL_RAMP, // Runs while held down; Ignores sensors
        EJECT_OUT, // Runs while held down; Button shared with CargoIntake
        SHOOT_BAY, // Checks if pneumatic down, then shoots
        SHOOT_ROCKET, // Checks if pneumatic up, then shoots
    }

    private enum SystemState
    {
        HOLDING, // Ramp runs until cargo reaches top
        EJECTING, // Ramp runs in reverse
        SHOOTING, // Timed or sensor??
        IDLING, // No cargo
    }

    private WantedState mWantedState = WantedState.MANUAL_RAMP;
    private SystemState mSystemState = SystemState.IDLING;

    private TalonSRX mRampMotor = null;
    private TalonSRX mShootMotorLeft = null;
    private TalonSRX mShootMotorRight = null;
    private Solenoid mFlipperSolenoid = null;

    private CargoChute()
    {
        boolean success = true;
        try
        {
            // Instantiate your hardware here
            mRampMotor = new TalonSRX(Constants.kRampMotorId);
            mShootMotorLeft = new TalonSRX(Constants.kShootMotorLeftId);
            mShootMotorRight = new TalonSRX(Constants.kShootMotorRightId);
            mFlipperSolenoid = new Solenoid(Constants.kFlipperSolenoidId);

            // TODO: Instantiate sensors
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
            synchronized (CargoChute.this)
            {
                mWantedState = WantedState.MANUAL_RAMP;
                mSystemState = SystemState.IDLING;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (CargoChute.this)
            {
                SystemState newState = defaultStateTransfer();
                switch (mSystemState)
                {
                    case HOLDING:
                        if (/* || (sensor || sensor*/) // TODO
                        {
                            mRampMotor.set(ControlMode.PercentOutput, Constants.kRampSpeed);
                        }
                        else
                        {
                            mRampMotor.set(ControlMode.PercentOutput, 0);
                        }
                        break;
                    case EJECTING:
                        if (newState != mSystemState)
                        {
                            mRampMotor.set(ControlMode.PercentOutput, -Constants.kRampSpeed);
                            mShootMotorLeft.set(ControlMode.PercentOutput, -Constants.kShootSpeed);
                            mShootMotorRight.set(ControlMode.PercentOutput, -Constants.kShootSpeed);
                        }
                        break;
                    case SHOOTING:
                        if (newState != mSystemState)
                        {
                            mShootMotorLeft.set(ControlMode.PercentOutput, Constants.kShootSpeed);
                            mShootMotorRight.set(ControlMode.PercentOutput, Constants.kShootSpeed);
                        }
                        else if ( not top sensor ) // FIXME: manual override? pressed twice?
                        {
                            mShootMotorLeft.set(ControlMode.PercentOutput, 0);
                            mShootMotorRight.set(ControlMode.PercentOutput, 0);
                            if (newState == mSystemState)
                                newState = SystemState.IDLING;
                        }
                        break;
                    case IDLING:
                        if (newState != mSystemState)
                        {
                            mRampMotor.set(ControlMode.PercentOutput, 0);
                            mShootMotorLeft.set(ControlMode.PercentOutput, 0);
                            mShootMotorRight.set(ControlMode.PercentOutput, 0);
                        }
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
            synchronized (CargoChute.this)
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
            case MANUAL_RAMP:
                // TODO: implement manual override
                newState = SystemState.HOLDING;
                break;
            case EJECT_OUT:
                newState = SystemState.EJECTING;
                break;
            case SHOOT_BAY:
                newState = SystemState.SHOOTING;
                break;
            case SHOOT_ROCKET:
                newState = SystemState.SHOOTING;
                break;
            default:
                newState = SystemState.HOLDING;
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
    }

    @Override
    public void stop()
    {
        // Stop your hardware here
        mWantedState = WantedState.MANUAL_RAMP;
        mSystemState = SystemState.IDLING;
    }
}
