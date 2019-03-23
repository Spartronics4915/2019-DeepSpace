package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.lib.util.CANProbe;
import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.drivers.TalonSRXFactory;
import com.spartronics4915.lib.drivers.A21IRSensor;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AnalogInput;
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

    public enum WantedState
    {
        RAMP_MANUAL, HOLD_MANUAL, BRING_BALL_TO_TOP, EJECT_BACK, LOWER, RAISE, SHOOT_ROCKET, SHOOT_BAY
    }

    private enum SystemState
    {
        RAMPING, HOLDING, EJECTING, LOWERING, RAISING, SHOOTING_ROCKET, SHOOTING_BAY
    }

    private WantedState mWantedState = WantedState.LOWER;
    private SystemState mSystemState = SystemState.LOWERING;

    private TalonSRX mRampMotor = null;
    private TalonSRX mRampMotorSlave = null;
    private Solenoid mRampSolenoid = null;
    private A21IRSensor mRampSensor = null;

    private Timer mCargoTimer = new Timer();
    private boolean mIsShootingBay;

    private boolean mStateChanged;

    private CargoChute()
    {
        boolean success = false;
        try
        {
            if (!CANProbe.getInstance().validatePCMId(Constants.kCargoHatchArmPCMId))
                throw new RuntimeException("CargoChute PCM isn't on the CAN bus!");

            mRampMotor = TalonSRXFactory.createDefaultTalon(Constants.kRampMotorId);
            mRampMotorSlave = TalonSRXFactory.createPermanentSlaveTalon(Constants.kRampMotorSlaveId,
                    Constants.kRampMotorId);
            mRampMotorSlave.setInverted(false);
            mRampSolenoid = new Solenoid(Constants.kCargoHatchArmPCMId, Constants.kRampSolenoidId);
            mRampSensor = new A21IRSensor(Constants.kRampSensorId);
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
            synchronized (CargoChute.this)
            {
                mStateChanged = true;
                mWantedState = WantedState.LOWER;
                mSystemState = SystemState.LOWERING;
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
                    case RAMPING:
                        if (mStateChanged)
                            mRampMotor.set(ControlMode.PercentOutput, Constants.kRampSpeed);
                        if (ballInPosition() && !isInManual() && newState == mSystemState)
                        {
                            newState = SystemState.HOLDING;
                            logNotice("Got a ball");
                        }
                        break;
                    case HOLDING:
                        if (mStateChanged)
                            mRampMotor.set(ControlMode.PercentOutput, 0.0);
                        if (!ballInPosition() && !isInManual() && newState == mSystemState)
                        {
                            newState = SystemState.RAMPING;
                            logNotice("No more ball");
                        }
                        break;
                    case EJECTING:
                        if (mStateChanged)
                            mRampMotor.set(ControlMode.PercentOutput, -Constants.kRampSpeed);
                        break;
                    case LOWERING:
                        if (mStateChanged)
                        {
                            mRampSolenoid.set(Constants.kRampSolenoidRetract);
                            mRampMotor.set(ControlMode.PercentOutput, 0.0);
                        }
                        break;
                    case RAISING:
                        if (mStateChanged)
                        {
                            mRampSolenoid.set(Constants.kRampSolenoidExtend);
                            mRampMotor.set(ControlMode.PercentOutput, 0.0);
                        }
                        break;
                    case SHOOTING_ROCKET:
                        if (mStateChanged)
                        {
                            mRampSolenoid.set(Constants.kRampSolenoidRetract);
                            mCargoTimer.start();
                            mRampMotor.set(ControlMode.PercentOutput, Constants.kRampSpeed);
                        }
                        if (mCargoTimer.hasPeriodPassed(Constants.kShootTime) && newState == mSystemState)
                            newState = SystemState.HOLDING;
                        break;
                    case SHOOTING_BAY:
                        if (mStateChanged)
                        {
                            mIsShootingBay = false;
                            mRampSolenoid.set(Constants.kRampSolenoidExtend);
                            mCargoTimer.start();
                            // Waits so that the solenoid can get up

                        }
                        if (mCargoTimer.hasPeriodPassed(Constants.kChuteHighExtendTime) && !mIsShootingBay)
                        {
                            mRampMotor.set(ControlMode.PercentOutput, Constants.kRampSpeed);
                            mIsShootingBay = true;
                        }
                        if (mCargoTimer.hasPeriodPassed(Constants.kShootTime) && newState == mSystemState)
                            newState = SystemState.HOLDING;
                        break;
                    default:
                        logError("Unhandled system state!");
                }
                if (newState != mSystemState)
                {
                    mCargoTimer.stop();
                    mCargoTimer.reset();
                    mStateChanged = true;
                }
                else
                    mStateChanged = false;

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

    private boolean isInManual()
    {
        return mWantedState == WantedState.RAMP_MANUAL || mWantedState == WantedState.HOLD_MANUAL;
    }

    private boolean ballInPosition()
    {
        return mRampSensor.getVoltage() >= Constants.kMinBallInChuteVoltage;
    }

    public boolean isRampRunning()
    {
        return mSystemState == SystemState.RAMPING
                || mSystemState == SystemState.EJECTING
                || mSystemState == SystemState.SHOOTING_BAY
                || mSystemState == SystemState.SHOOTING_ROCKET;
    }

    private SystemState defaultStateTransfer()
    {
        SystemState newState = mSystemState;
        switch (mWantedState)
        {
            case RAMP_MANUAL:
                newState = SystemState.RAMPING;
                break;
            case HOLD_MANUAL:
                newState = SystemState.HOLDING;
                break;
            case EJECT_BACK:
                newState = SystemState.EJECTING;
                break;
            case BRING_BALL_TO_TOP:
                if (mSystemState != SystemState.RAMPING || mSystemState != SystemState.HOLDING)
                    newState = SystemState.HOLDING;
                break;
            case LOWER:
                newState = SystemState.LOWERING;
                break;
            case RAISE:
                newState = SystemState.RAISING;
                break;
            case SHOOT_ROCKET:
                newState = SystemState.SHOOTING_ROCKET;
                break;
            case SHOOT_BAY:
                newState = SystemState.SHOOTING_BAY;
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
            case RAMP_MANUAL:
                return mSystemState == SystemState.RAMPING;
            case HOLD_MANUAL:
                return mSystemState == SystemState.HOLDING;
            case EJECT_BACK:
                return mSystemState == SystemState.EJECTING && !ballInPosition(); // i'm slightly unclear on how atTarget works but it seems like this should check the CargoIntake sensor
            case BRING_BALL_TO_TOP:
                return mSystemState == SystemState.HOLDING && ballInPosition();
            case LOWER:
                return mSystemState == SystemState.LOWERING;
            case RAISE:
                return mSystemState == SystemState.RAISING;
            case SHOOT_ROCKET:
                return mSystemState == SystemState.SHOOTING_ROCKET && mCargoTimer.hasPeriodPassed(Constants.kShootTime);
            case SHOOT_BAY:
                return mSystemState == SystemState.SHOOTING_BAY && mCargoTimer.hasPeriodPassed(Constants.kShootTime);
            default:
                logError("CargoChute atTarget for unknown WantedState: " + mWantedState);
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
        logNotice("Beginning CargoChute system check:");

        logNotice("Beginning ramp check.");
        try
        {
            logNotice("Running RampMotor at ramping speed for five seconds: ");
            mRampMotor.set(ControlMode.PercentOutput, Constants.kRampSpeed);
            Timer.delay(5);
            logNotice("Done.");
            logNotice("Running RampMotor at zero speed for three seconds: ");
            mRampMotor.set(ControlMode.PercentOutput, 0.0);
            Timer.delay(3);
            logNotice("Done.");
            logNotice("Running RampMotor at reverse ramping speed for five seconds: ");
            mRampMotor.set(ControlMode.PercentOutput, -Constants.kRampSpeed);
            Timer.delay(5);
            logNotice("Done.");
            mRampMotor.set(ControlMode.PercentOutput, 0.0);
        }
        catch (Exception e)
        {
            logException("Did not pass ramp check: ", e);
            return false;
        }
        logNotice("Ramp check complete.");

        logNotice("Beginning pneumatic check: "); // TODO: oh chute this might hurt the intake arm
        try
        {
            logNotice("Sending ramp pneumatics up for three seconds: ");
            mRampSolenoid.set(Constants.kRampSolenoidExtend);
            Timer.delay(3);
            logNotice("Done.");
            logNotice("Sending ramp pneumatics down for three seconds: ");
            mRampSolenoid.set(Constants.kRampSolenoidRetract);
            Timer.delay(3);
            logNotice("Done.");
            logNotice("Sending ramp pneumatics up for three seconds: ");
            mRampSolenoid.set(Constants.kRampSolenoidExtend);
            Timer.delay(3);
            logNotice("Done.");
            logNotice("Sending ramp pneumatics down for three seconds: ");
            mRampSolenoid.set(Constants.kRampSolenoidRetract);
            Timer.delay(3);
            logNotice("Done.");
        }
        catch (Exception e)
        {
            logException("Did not pass pneumatic check: ", e);
            return false;
        }
        logNotice("Pneumatic check complete.");

        logNotice("CargoChute system check complete.");
        return true;
    }

    @Override
    public void outputTelemetry()
    {
        dashboardPutState(mSystemState.toString());
        dashboardPutWantedState(mWantedState.toString());
        dashboardPutBoolean("mRampSolenoid extended: ", mRampSolenoid.get());
        dashboardPutNumber("mRampMotor speed: ", mRampMotor.getMotorOutputPercent());
        dashboardPutNumber("mRampSensor voltage: ", mRampSensor.getVoltage());
        dashboardPutBoolean("Ball in position: ", ballInPosition());
    }

    @Override
    public void stop()
    {
        // Stop your hardware here
        mWantedState = WantedState.LOWER;
        mSystemState = SystemState.HOLDING;
        mRampMotor.set(ControlMode.PercentOutput, 0.0);
        mRampSolenoid.set(Constants.kRampSolenoidRetract);
    }
}
