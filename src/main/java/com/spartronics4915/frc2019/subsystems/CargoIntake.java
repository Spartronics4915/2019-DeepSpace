/* TODO
 *
 * Change motor names to align with the front of the robot
 */

package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.lib.drivers.TalonSRXFactory;
import com.spartronics4915.lib.util.CANProbe;
import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class CargoIntake extends Subsystem
{

    private static CargoIntake mInstance = null;

    public static CargoIntake getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new CargoIntake();
        }
        return mInstance;
    }

    public enum WantedState
    {
        HOLD, ARM_DOWN, INTAKE, EJECT, CLIMB, MOTORS_STOP
    }

    private enum SystemState
    {
        HOLDING, ARM_DOWNING, INTAKING, EJECTING, CLIMBING, MOTORS_STOPPING
    }

    private WantedState mWantedState = WantedState.HOLD;
    private SystemState mSystemState = SystemState.HOLDING;

    private Solenoid mSolenoid = null;
    private Solenoid mSolenoidClimb = null;
    private TalonSRX mMotorLeft = null; // right and left are switched?
    private TalonSRX mMotorRight = null;

    private boolean mStateChanged;

    private CargoIntake()
    {
        boolean success = false; // IR sensor anolog port 6 to detect cargo going into chute. Used by chute as well.
        try
        {
            if (!CANProbe.getInstance().validatePCMId(Constants.kCargoHatchArmPCMId))
                throw new RuntimeException("CargoIntake PCM isn't on the CAN bus!");

            mMotorLeft = TalonSRXFactory.createDefaultTalon(Constants.kCargoIntakeMotorRight);
            mMotorRight = TalonSRXFactory.createDefaultTalon(Constants.kCargoIntakeMotorLeft);
            mSolenoid = new Solenoid(Constants.kCargoHatchArmPCMId, Constants.kCargoIntakeSolenoid);
            mSolenoidClimb = new Solenoid(Constants.kCargoHatchArmPCMId, Constants.kCargoIntakeSolenoidClimb);
            success = true;
        }
        catch (Exception e)
        {
            success = false;
            logException("Couldn't instantiate hardware: ", e);
        }

        logInitialized(success);
    }

    private final ILoop mLoop = new ILoop()
    {

        @Override
        public void onStart(double timestamp)
        {
            synchronized (CargoIntake.this)
            {
                mStateChanged = true;
                mWantedState = WantedState.HOLD;
                mSystemState = SystemState.HOLDING;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (CargoIntake.this)
            {
                SystemState newState = defaultStateTransfer();
                switch (mSystemState)
                {
                    case HOLDING://DS5
                        if (mStateChanged)
                        {
                            mMotorLeft.set(ControlMode.PercentOutput, 0.0);
                            mMotorRight.set(ControlMode.PercentOutput, 0.0);
                            setSolenoidsToUp();
                        }
                        break;
                    case MOTORS_STOPPING:
                        if (mStateChanged)
                        {
                            mMotorLeft.set(ControlMode.PercentOutput, 0.0);
                            mMotorRight.set(ControlMode.PercentOutput, 0.0);
                        }
                        break;
                    case ARM_DOWNING:
                        if (mStateChanged)
                        {
                            setSolenoidsToDown();
                            mMotorLeft.set(ControlMode.PercentOutput, 0.0);
                            mMotorRight.set(ControlMode.PercentOutput, 0.0);
                        }
                        break;
                    case INTAKING://BB2
                        if (mStateChanged)
                        {
                            setSolenoidsToDown();
                            mMotorLeft.set(ControlMode.PercentOutput, Constants.kCargoIntakeSpeed);
                            mMotorRight.set(ControlMode.PercentOutput, Constants.kCargoIntakeSpeed);
                        }
                        break;
                    case EJECTING://BB3
                        if (mStateChanged)
                        {
                            setSolenoidsToDown();
                            mMotorLeft.set(ControlMode.PercentOutput, Constants.kCargoEjectSpeed);
                            mMotorRight.set(ControlMode.PercentOutput, Constants.kCargoEjectSpeed);
                        }
                        break;
                    case CLIMBING:
                        if (mStateChanged)
                        {
                            mMotorLeft.set(ControlMode.PercentOutput, Constants.kCargoIntakeClimbSpeed);
                            mMotorRight.set(ControlMode.PercentOutput, Constants.kCargoIntakeClimbSpeed);
                            mSolenoid.set(Constants.kCargoIntakeSolenoidExtend);
                            mSolenoidClimb.set(Constants.kCargoIntakeSolenoidExtend);
                        }
                        break;
                    default:
                        logError("Unhandled system state!");
                }
                if (newState != mSystemState)
                    mStateChanged = true;
                else
                    mStateChanged = false;
                mSystemState = newState;
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            synchronized (CargoIntake.this)
            {
                stop();
            }
        }
    };

    private void setSolenoidsToUp()
    {
        mSolenoid.set(Constants.kCargoIntakeSolenoidRetract);
        mSolenoidClimb.set(Constants.kCargoIntakeSolenoidRetract);
    }

    private void setSolenoidsToDown()
    {
        mSolenoid.set(Constants.kCargoIntakeSolenoidExtend);
        mSolenoidClimb.set(Constants.kCargoIntakeSolenoidRetract);
    }

    private SystemState defaultStateTransfer()
    {
        SystemState newState = mSystemState;
        switch (mWantedState)
        {
            case HOLD:
                newState = SystemState.HOLDING;
                break;
            case MOTORS_STOP:
                newState = SystemState.MOTORS_STOPPING;
                break;
            case ARM_DOWN:
                newState = SystemState.ARM_DOWNING;
                break;
            case INTAKE:
                newState = SystemState.INTAKING;
                break;
            case EJECT:
                newState = SystemState.EJECTING;
                break;
            case CLIMB:
                newState = SystemState.CLIMBING;
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
        switch (mWantedState)
        {
            case HOLD:
                return mSystemState == SystemState.HOLDING;
            case MOTORS_STOP:
                return mSystemState == SystemState.MOTORS_STOPPING;
            case ARM_DOWN:
                return mSystemState == SystemState.ARM_DOWNING;
            case INTAKE:
                return mSystemState == SystemState.INTAKING;
            case EJECT:
                return mSystemState == SystemState.EJECTING;
            case CLIMB:
                return mSystemState == SystemState.CLIMBING;
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
        logNotice("Starting CargoIntake Solenoid Check");
        try
        {
            logNotice("Extending solenoids for 4 seconds");
            mSolenoid.set(Constants.kCargoIntakeSolenoidExtend);
            mSolenoidClimb.set(Constants.kCargoIntakeSolenoidExtend);
            Timer.delay(4);
            logNotice("Retracting solenoids for 4 seconds");
            setSolenoidsToUp();
            Timer.delay(4);
            logNotice("CargoIntake Solenoid Check End");
            logNotice("Running motors at 50% for 2 seconds");
            mMotorLeft.set(ControlMode.PercentOutput, Constants.kCargoIntakeSpeed);
            mMotorRight.set(ControlMode.PercentOutput, Constants.kCargoIntakeSpeed);
            Timer.delay(2);
            logNotice("Running motors at 0% for 2 seconds");
            mMotorLeft.set(ControlMode.PercentOutput, 0.0);
            mMotorRight.set(ControlMode.PercentOutput, 0.0);
            Timer.delay(2);
            logNotice("Running motors at -50% for 2 seconds");
            mMotorLeft.set(ControlMode.PercentOutput, Constants.kCargoEjectSpeed);
            mMotorRight.set(ControlMode.PercentOutput, Constants.kCargoEjectSpeed);
            Timer.delay(2);
            mMotorLeft.set(ControlMode.PercentOutput, 0.0);
            mMotorRight.set(ControlMode.PercentOutput, 0.0);
            logNotice("CargoIntake Motor Check End");
        }
        catch (Exception e)
        {
            logException("Trouble instantiating hardware: ", e);
            return false;
        }
        return true;
    }

    @Override
    public void outputTelemetry()
    {
        dashboardPutState(mSystemState.toString());
        dashboardPutWantedState(mWantedState.toString());
        dashboardPutBoolean("mSolenoid Extended", mSolenoid.get());
        dashboardPutBoolean("mSolenoidClimb Extended", mSolenoidClimb.get());
        dashboardPutNumber("mMotor1 Speed", mMotorLeft.getMotorOutputPercent());
        dashboardPutNumber("mMotor2 Speed", mMotorRight.getMotorOutputPercent());
    }

    @Override
    public void stop()
    {
        setSolenoidsToUp();
        mMotorLeft.set(ControlMode.PercentOutput, 0.0);
        mMotorRight.set(ControlMode.PercentOutput, 0.0);
    }
}