package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.ControlBoard;
import com.spartronics4915.lib.drivers.A21IRSensor;
import com.spartronics4915.lib.drivers.IRSensor;
import com.spartronics4915.lib.util.CANProbe;
import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;

import edu.wpi.first.hal.sim.mockdata.PCMDataJNI;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Climber extends Subsystem
{

    private static Climber mInstance = null;

    public static Climber getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new Climber();
        }
        return mInstance;
    }

    public enum WantedState
    {
        DISABLE, CLIMB, RETRACT_FRONT_STRUTS, RETRACT_REAR_STRUTS
    }

    private enum SystemState
    {
        DISABLING, CLIMBING, RETRACTING_FRONT_STRUTS, RETRACTING_REAR_STRUTS
    }

    private WantedState mWantedState = WantedState.DISABLE;
    private SystemState mSystemState = SystemState.DISABLING;
    private DoubleSolenoid mFrontLeftClimberSolenoid = null;
    private DoubleSolenoid mFrontRightClimberSolenoid = null;
    private DoubleSolenoid mRearLeftClimberSolenoid = null;
    private DoubleSolenoid mRearRightClimberSolenoid = null;
    public IRSensor mClimberFrontIRSensor = null;
    public IRSensor mClimberRearIRSensor = null;

    private Climber()
    {
        boolean success = false;
        try
        {
            if (!CANProbe.getInstance().validatePCMId(Constants.kClimberPCMId))
                throw new RuntimeException("Climber PCM isn't on the CAN bus!");

            mFrontLeftClimberSolenoid = new DoubleSolenoid(Constants.kClimberPCMId, Constants.kFrontLeftSolenoidId1,
                    Constants.kFrontLeftSolenoidId2);
            mFrontRightClimberSolenoid = new DoubleSolenoid(Constants.kClimberPCMId, Constants.kFrontRightSolenoidId1,
                    Constants.kFrontRightSolenoidId2);
            mRearLeftClimberSolenoid = new DoubleSolenoid(Constants.kClimberPCMId, Constants.kRearLeftSolenoidId1,
                    Constants.kRearLeftSolenoid2);
            mRearRightClimberSolenoid = new DoubleSolenoid(Constants.kClimberPCMId, Constants.kRearRightSolenoidId1,
                    Constants.kRearRightSolenoidId2);
            mClimberFrontIRSensor = new A21IRSensor(Constants.kClimberFrontIRSensorID);
            mClimberRearIRSensor = new A21IRSensor(Constants.kClimberRearIRSensorID);
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

        private boolean mStateChanged = true;
        private Timer mStateChangedTimer = new Timer();

        @Override
        public void onStart(double timestamp)
        {
            synchronized (Climber.this)
            {
                mStateChanged = true;
                mWantedState = WantedState.DISABLE;
                mSystemState = SystemState.DISABLING;
                mStateChangedTimer.reset();
                mStateChangedTimer.start();
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (Climber.this)
            {
                SystemState newState = defaultStateTransfer();
                switch (mSystemState)
                {
                    case DISABLING: // Only used when starting, cannot be transitioned into
                        if (mStateChanged)
                        {
                            mFrontLeftClimberSolenoid.set(Value.kReverse);
                            mFrontRightClimberSolenoid.set(Value.kReverse);
                            mRearLeftClimberSolenoid.set(Value.kReverse);
                            mRearRightClimberSolenoid.set(Value.kReverse);
                        }
                        break;

                    case CLIMBING:
                        // Struts will extend from their dormant position to allow the robot to reach the height required to get to L3
                        // Must be done when robot is flushed with L3 (Done with distance sensors and a backup encoder reading)
                        if (mStateChanged)
                        {
                            mRearLeftClimberSolenoid.set(Value.kForward);
                            mRearRightClimberSolenoid.set(Value.kForward);
                        }
                        if (mStateChangedTimer.hasPeriodPassed(Constants.kClimberFrontSolenoidDelay))
                        {
                            mFrontLeftClimberSolenoid.set(Value.kForward);
                            mFrontRightClimberSolenoid.set(Value.kForward);

                            mStateChangedTimer.stop();
                            mStateChangedTimer.reset();
                        }
                        break;

                    case RETRACTING_FRONT_STRUTS:
                        // Solenoids from the front struts will retract when they become flushed with L3
                        // Done with distance sensors and backup driver vision
                        if (mStateChanged)
                        {
                            mFrontLeftClimberSolenoid.set(Value.kReverse);
                            mFrontRightClimberSolenoid.set(Value.kReverse);
                        }
                        break;

                    case RETRACTING_REAR_STRUTS:
                        // Solenoids from the rear struts will retract when the robot can support its own weight on L3
                        // Done primarily with driver vision, but distance sensor might be used
                        if (mStateChanged)
                        {
                            mRearLeftClimberSolenoid.set(Value.kReverse);
                            mRearRightClimberSolenoid.set(Value.kReverse);
                        }
                        break;

                    default:
                        logError("Unhandled system state!");
                }
                if (newState != mSystemState)
                {
                    mStateChanged = true;

                    mStateChangedTimer.reset();
                    mStateChangedTimer.start();
                }
                else
                    mStateChanged = false;
                mSystemState = newState;
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            synchronized (Climber.this)
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
            case DISABLE:
                newState = SystemState.DISABLING;
                break;
            case CLIMB:
                newState = SystemState.CLIMBING;
                break;
            case RETRACT_FRONT_STRUTS:
                newState = SystemState.RETRACTING_FRONT_STRUTS;
                break;
            case RETRACT_REAR_STRUTS:
                newState = SystemState.RETRACTING_REAR_STRUTS;
                break;
            default:
                newState = SystemState.DISABLING;
                logNotice("Robot is in an unhandled WantedState!");
                break;
        }
        return newState;
    }

    public synchronized void setWantedState(WantedState wantedState)
    {
        mWantedState = wantedState;
    }

    public boolean isClimbing()
    {
        return !(mWantedState == Climber.WantedState.DISABLE);
    }

    public boolean frontSensorsInRange()
    {
        return mClimberFrontIRSensor.isTargetInVoltageRange(Constants.kClimberSensorFrontMinVoltage,
            Constants.kClimberSensorFrontMinVoltage+1);
    }

    public boolean rearSensorsInRange()
    {
        return mClimberRearIRSensor.isTargetInVoltageRange(Constants.kClimberSensorRearMinVoltage,
            Constants.kClimberSensorRearMinVoltage+1);
    }

    public synchronized boolean atTarget()
    {
        switch (mWantedState)
        {
            case DISABLE:
                return mSystemState == SystemState.DISABLING;
            case CLIMB:
                return mSystemState == SystemState.CLIMBING;
            case RETRACT_FRONT_STRUTS:
                if (frontSensorsInRange())
                    return mSystemState == SystemState.RETRACTING_FRONT_STRUTS;
                else
                    return false;
            case RETRACT_REAR_STRUTS:
                if (rearSensorsInRange())
                    return mSystemState == SystemState.RETRACTING_REAR_STRUTS;
                else
                    return false;
            default:
                logError("Climber in unhandled Wanted State!");
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
        logNotice("Lifting for 5 Seconds");
        try
        {
            mFrontLeftClimberSolenoid.set(Value.kForward);
            mFrontRightClimberSolenoid.set(Value.kForward);
            mRearLeftClimberSolenoid.set(Value.kForward);
            mRearRightClimberSolenoid.set(Value.kForward);
            Timer.delay(5);
            mFrontLeftClimberSolenoid.set(Value.kReverse);
            mFrontRightClimberSolenoid.set(Value.kReverse);
            mRearLeftClimberSolenoid.set(Value.kReverse);
            mRearRightClimberSolenoid.set(Value.kReverse);
        }
        catch (Exception e)
        {
            logException("Did not pass lifting check: ", e);
            return false;
        }

        logNotice("Testing IR Sensors");
        try
        {
            Timer.delay(2);
            mClimberFrontIRSensor.getVoltage();
            logNotice("Downward Front IR Sensor Voltage is " + mClimberFrontIRSensor.getVoltage());
            Timer.delay(2);
            mClimberRearIRSensor.getVoltage();
            logNotice("Downward Rear IR Sensor Voltage is " + mClimberRearIRSensor.getVoltage());
            Timer.delay(2);
        }
        catch (Exception e)
        {
            logException("Did not pass IR sensor check: ", e);
            return false;
        }

        return true;
    }

    @Override
    public void outputTelemetry()
    {
        dashboardPutState(mSystemState.toString());
        dashboardPutWantedState(mWantedState.toString());
        dashboardPutNumber("Forward sensor voltage: ", mClimberFrontIRSensor.getVoltage());
        dashboardPutBoolean("Forward sensor in range: ", frontSensorsInRange());
        dashboardPutNumber("Rear sensor voltage: ", mClimberRearIRSensor.getVoltage());
        dashboardPutBoolean("Rear sensor in range: ", rearSensorsInRange());
    }

    @Override
    public void stop()
    {
        // Stop your hardware here
    }
}
