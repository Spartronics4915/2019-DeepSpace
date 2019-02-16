/* TODOS
 * ✔ Mechanics removed the ShootMotors - remove these
 * ✔ Read through the CheckSystem method and ask questions
 * ✔ Use TalonSRXFactory to instansiate Talons (ask Declan)
 * ✔ Instantiate sensors
 * ✔ Use the sensors in places
 * ✔ Don't keep shooting states running forever?
 * ✔ Fill out atTarget()
 * ✔ Fill out outputTelemetry()
 * the chute can break the arm now. we gotta write code to stop that from happening.
 * - Make sure that CargoChute lowers the arm before extending to the rocket height, and additionally...
 *     - Lowers the chute immediately after successfully shooting the cargo, but...
 *     - Does not allow for the arm to be retracted during that time.
 * - Make sure that anywhere _using_ the cargo chute or arm won't break either
 *     - Places include CheckSystem and and some superstructure?
 * ✔ Mechanics added the ShootMotors - add these
 * ✔ Mechanics removed the ShootMotors - remove these
 *
 * T E S T
 * - Both SHOOTs should... shoot. Solenoids and the ejecting wheels should work too.
 * - Need to integrate with CargoIntake to prevent arm damage. (!!!thisisimportant!!!)
 * - A21IR sensor should give us correct data (Austin has checked his A21s, but it's really really good to make sure)
 * - BRING_BALL_TO_TOP should correctly transition with IR sensors (make sure readings are accurate beforehand)
 * - MANUALs should correctly override
 *     - If RampMotor moving at all go into MANUAL_HOLDING
 *     - If RampMotor not moving go into MANUAL_RAMPING
 *     - MUST not be overriden ~~by the A21 sensor detecting a ball~~ scratch that for now we afaik we aren't going to detect cargo
 *     - Should switch with a click of a button
 * - There should be a quick transition so that ramp motors do not go from full speed ahead to full speed behind
 *
 * After all this is completed: code review
 */

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

    public enum WantedState
    {
        RAMP_MANUAL, HOLD_MANUAL, BRING_BALL_TO_TOP, EJECT_BACK, LOWER, SHOOT_BAY, SHOOT_ROCKET,
    }

    private enum SystemState
    {
        RAMPING, HOLDING, EJECTING, LOWERING, SHOOTING_BAY, SHOOTING_ROCKET,
    }

    private WantedState mWantedState = WantedState.BRING_BALL_TO_TOP;
    private SystemState mSystemState = SystemState.HOLDING;

    private TalonSRX mRampMotor = null;
    private Solenoid mRampSolenoid = null;
    private A21IRSensor mRampSensor = null;

    private Timer mCargoTimer = new Timer();

    private boolean mStateChanged;

    private CargoChute()
    {
        boolean success = true;
        try
        {
            if (!CANProbe.getInstance().validatePCMId(Constants.kCargoHatchArmPCMId)) throw new RuntimeException("CargoChute PCM isn't on the CAN bus!");

            mRampMotor = TalonSRXFactory.createDefaultTalon(Constants.kRampMotorId);
            mRampSolenoid = new Solenoid(Constants.kCargoHatchArmPCMId, Constants.kRampSolenoidId);
            mRampSensor = new A21IRSensor(Constants.kRampSensorId);
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
                            mRampMotor.set(ControlMode.PercentOutput, Constants.kRampSpeed);
                        if (ballInPosition() && !isInManual() && newState == mSystemState)
                            newState = SystemState.HOLDING;
                        break;
                    case HOLDING:
                        if (mStateChanged)
                            mRampMotor.set(ControlMode.PercentOutput, 0.0);
                        if (!ballInPosition() && !isInManual() && newState == mSystemState)
                            newState = SystemState.RAMPING;
                        break;
                    case EJECTING: // We stop the motors for 1 second, then reverse them (this lessens the jerk)
                        if (mStateChanged)
                        {
                            /*
                            mCargoTimer.start();
                            mRampMotor.set(ControlMode.PercentOutput, 0.0);
                        }
                        if (mCargoTimer.hasPeriodPassed(Constants.kTransitionTime)) // Does the period still pass if the timer is stopped?
                        {
                            mCargoTimer.stop();
                            mCargoTimer.reset(); // I believe we need this reset here per above comment
                            */
                            mRampMotor.set(ControlMode.PercentOutput, -Constants.kRampSpeed);
                        }
                        break;
                    case LOWERING:
                        if (mStateChanged)
                            mRampSolenoid.set(Constants.kRampSolenoidRetract);
                        break;
                    case SHOOTING_BAY:
                        if (mStateChanged)
                        {
                            mRampSolenoid.set(Constants.kRampSolenoidRetract);
                            mCargoTimer.start();
                            mRampMotor.set(ControlMode.PercentOutput, Constants.kRampSpeed);
                        }
                        if (mCargoTimer.hasPeriodPassed(Constants.kShootTime) && newState == mSystemState)
                            newState = SystemState.HOLDING;
                        break;
                    case SHOOTING_ROCKET:
                        if (mStateChanged)
                        {
                            mRampSolenoid.set(Constants.kRampSolenoidExtend);
                            mCargoTimer.start();
                            mRampMotor.set(ControlMode.PercentOutput, Constants.kRampSpeed);
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
        return mRampSensor.getDistance() <= Constants.kMaxChuteBallDistanceThreshold;
    }

    public boolean isRampRunning()
    {
        return mSystemState == SystemState.RAMPING || mSystemState == SystemState.EJECTING || mSystemState == SystemState.SHOOTING_BAY || mSystemState == SystemState.SHOOTING_ROCKET;
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
        switch (mWantedState)
        {
            case RAMP_MANUAL:
                return mSystemState == SystemState.RAMPING;
            case HOLD_MANUAL:
                return mSystemState == SystemState.HOLDING;
            case EJECT_BACK:
               return mSystemState == SystemState.EJECTING && !ballInPosition(); // TODO: i'm slightly unclear on how atTarget works but it seems like this should check the CargoIntake sensor
            case BRING_BALL_TO_TOP:
                return mSystemState == SystemState.HOLDING && ballInPosition();
            case LOWER:
                return mSystemState == SystemState.LOWERING;
            case SHOOT_BAY:
                return mSystemState == SystemState.SHOOTING_BAY && mCargoTimer.hasPeriodPassed(Constants.kShootTime);
            case SHOOT_ROCKET:
                return mSystemState == SystemState.SHOOTING_ROCKET;
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

        logNotice("Beginning pneumatic check: "); // TODO: oh shoot this might hurt the intake arm
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
        dashboardPutBoolean("mRampSolenoid extended: ", !mRampSolenoid.get()); // Yes it is reverse
        dashboardPutNumber("mRampMotor speed: ", mRampMotor.getMotorOutputPercent());
        dashboardPutNumber("mRampSensor distance: ", mRampSensor.getDistance());
    }

    @Override
    public void stop()
    {
        // Stop your hardware here
        mWantedState = WantedState.BRING_BALL_TO_TOP;
        mSystemState = SystemState.HOLDING;
        mRampMotor.set(ControlMode.PercentOutput, 0.0);
        mRampSolenoid.set(Constants.kRampSolenoidRetract);
    }
}