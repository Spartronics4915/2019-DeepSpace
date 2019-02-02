/* TODOS
 * ✔ Mechanics removed the ShootMotors - remove these
 * ✔ Read through the CheckSystem method and ask questions
 * ✔ Use TalonSRXFactory to instansiate Talons (ask Declan)
 * ✔ TODO: Instantiate sensors
 * Use the sensors in places
 * Don't keep shooting states running forever?
 */

package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;
import com.spartronics4915.lib.util.Logger;

import com.spartronics4915.lib.drivers.IRSensor;

import edu.wpi.first.wpilibj.AnalogInput;

import com.spartronics4915.lib.drivers.TalonSRXFactory;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
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
        RAMP_MANUAL, HOLD_MANUAL, BRING_BALL_TO_TOP, EJECT_BACK, SHOOT_BAY, SHOOT_ROCKET,
    }

    private enum SystemState
    {
        RAMPING, HOLDING, EJECTING, SHOOTING_BAY, SHOOTING_ROCKET,
    }

    private WantedState mWantedState = WantedState.BRING_BALL_TO_TOP;
    private SystemState mSystemState = SystemState.HOLDING;

    //Motor
    private TalonSRX mRampMotor = null;
    
    //Solenoid
    private Solenoid mFlipperSolenoid = null;

    private static final boolean mSolenoidExtend = true;
    private static final boolean mSolenoidRetract = false;
    private boolean mSolenoidStatus = false;
    private boolean mStateChanged;

    //Sensor
    private IRSensor mRampSensor = null;

    private boolean isInManual()
    {
        return mWantedState == WantedState.RAMP_MANUAL || mWantedState == WantedState.HOLD_MANUAL;
    }

    private boolean ballInPosition()
    {
        return mRampSensor.getDistance() == 1;
    }

    private boolean solenoidIsOut() {
        return mFlipperSolenoid.get() == Constants.kSolenoidOut;
    }

    private boolean solenoidIsIn() {
        return mFlipperSolenoid.get() == Constants.kSolenoidIn;
    }


    private CargoChute()
    {
        boolean success = true;
        try
        {
            // Instantiate your hardware here
            mRampMotor = TalonSRXFactory.createDefaultTalon(Constants.kRampMotorId);
            mFlipperSolenoid = new Solenoid(1, Constants.kFlipperSolenoidId);

            // TODO: Instantiate sensor(s)
            mRampSensor = new IRSensor(Constants.kRampSensorId);
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
                mWantedState = WantedState.BRING_BALL_TO_TOP;
                mSystemState = SystemState.HOLDING;
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
                        {
                            mRampMotor.set(ControlMode.PercentOutput, Constants.kRampSpeed);
                        }
                        if (true /* TODO: Ball in position */ && !isInManual() && mSystemState == newState)
                        {
                            newState = SystemState.HOLDING;
                        }
                        if (true  && !isInManual() && mSystemState == newState)
                        {

                            newState = SystemState.HOLDING;
                        }
                        break;
                    case HOLDING:
                        if (mStateChanged)
                        {
                            mRampMotor.set(ControlMode.PercentOutput, 0);
                        }
                        if (true /* TODO: Ball not in position */ && !isInManual() && mSystemState == newState)
                        {
                            newState = SystemState.RAMPING;
                        }
                        break;
                    case EJECTING:
                        if (mStateChanged)
                        {
                            mRampMotor.set(ControlMode.PercentOutput, -Constants.kRampSpeed);
                        }
                        break;
                    case SHOOTING_BAY: // don't keep running
                        if (mStateChanged)
                        {
                            if(solenoidIsOut()){
                                mFlipperSolenoid.set(Constants.kSolenoidIn);
                            }
                        }
                        break;
                    case SHOOTING_ROCKET: // don't keep running
                        if (mStateChanged)
                        {
                            if(solenoidIsIn()){
                                mFlipperSolenoid.set(Constants.kSolenoidOut);
                            }
                        }
                        break;
                    default:
                        logError("Unhandled system state!");
                }
                if (newState != mSystemState)
                {
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
                    newState = SystemState.HOLDING; // this checks if the SYstemState is in holding or ramping
                break;
            case SHOOT_BAY:
                newState = SystemState.SHOOTING_BAY;
                break;
            case SHOOT_ROCKET:
                newState = SystemState.SHOOTING_ROCKET;
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
        try
        {
            logNotice("Beginning CargoChute system check:");

            logNotice("Beginning ramp check.");

            logNotice("Running ramp at default speed for five seconds: ");
            mRampMotor.set(ControlMode.PercentOutput, Constants.kRampSpeed);
            Timer.delay(5);
            logNotice("Done.");

            logNotice("Running ramp at zero speed for three seconds: ");
            mRampMotor.set(ControlMode.PercentOutput, 0);
            Timer.delay(3);
            logNotice("Done.");
            
            logNotice("Running ramp at reverse default speed for five seconds: ");
            mRampMotor.set(ControlMode.PercentOutput, -Constants.kRampSpeed);
            Timer.delay(5);
            logNotice("Done.");
            mRampMotor.set(ControlMode.PercentOutput, 0);
            logNotice("Ramp check complete.");

            logNotice("Beginning pneumatic check: ");

            logNotice("Sending ramp pneumatics up for three seconds: ");
            mFlipperSolenoid.set(true);
            Timer.delay(3);
            logNotice("Done.");

            logNotice("Sending ramp pneumatics down for three seconds: ");
            mFlipperSolenoid.set(true);
            Timer.delay(3);
            logNotice("Done.");

            logNotice("Sending ramp pneumatics up for three seconds: ");
            mFlipperSolenoid.set(true);
            Timer.delay(3);
            logNotice("Done.");

            logNotice("Sending ramp pneumatics down for five seconds: ");
            mFlipperSolenoid.set(true);
            Timer.delay(3);
            logNotice("Done.");
            logNotice("Pneumatic check complete.");

            logNotice("CargoChute system check complete.");
        }
        catch (Exception e)
        {
            logException("Did not pass checkSystem: ", e);
            return false;
        }

        return true;
    }

    @Override
    public void outputTelemetry()
    {
    }

    @Override
    public void stop()
    {
        // Stop your hardware here
        mWantedState = WantedState.BRING_BALL_TO_TOP;
        mSystemState = SystemState.HOLDING;
        mRampMotor.set(ControlMode.PercentOutput, 0.0);
        mFlipperSolenoid.set(mSolenoidRetract);

    }
}
