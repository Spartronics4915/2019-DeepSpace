package com.spartronics4915.frc2019.subsystems;

import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class LearningSubsystem extends Subsystem
{

    private static LearningSubsystem mInstance = null;

    public static LearningSubsystem getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new LearningSubsystem();
        }
        return mInstance;
    }

    public enum WantedState
    {
        CLOSED, INTAKE,
    }

    private enum SystemState
    {
        CLOSING, INTAKING,
    }

    private WantedState mWantedState = WantedState.CLOSED;
    private SystemState mSystemState = SystemState.CLOSING;

    private TalonSRX mIntakeMotor = null;

    private LearningSubsystem()
    {
        boolean success = true;
        try
        {
            // Instantiate your hardware here
            mIntakeMotor = new TalonSRX(14);
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
            synchronized (LearningSubsystem.this)
            {
                mWantedState = WantedState.CLOSED;
                mSystemState = SystemState.CLOSING;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (LearningSubsystem.this)
            {
                SystemState newState = defaultStateTransfer();
                switch (mSystemState)
                {
                    case INTAKING:
                        mIntakeMotor.set(ControlMode.PercentOutput, 1.0);
                        break;
                    case CLOSING:
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
            synchronized (LearningSubsystem.this)
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
            case CLOSED:
                newState = SystemState.CLOSING;
                break;
            case INTAKE:
                newState = SystemState.INTAKING;
                break;
            default:
                newState = SystemState.CLOSING;
                break;
        }
        return newState;
    }

    public synchronized void setWantedState(WantedState wantedState)
    {
        mWantedState = wantedState;
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
        mIntakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

    // This method would not normally be in a subsystem (it would be in Robot), but
    // you need to edit it to get user input
    public void teleopPeriodic(Joystick joystick)
    {
        if (joystick.getRawButtonPressed(1))
        {
            this.setWantedState(WantedState.INTAKE);
        }
        else if (joystick.getRawButtonPressed(2))
        {
            this.setWantedState(WantedState.CLOSED);
            logNotice("foo");
        }
    }
}
