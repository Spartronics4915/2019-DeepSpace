//  Note: All timing is to be done from commands.
//  Previous states that used both the front and back sets of pneumatics have been deprecated,
//  as parallel commands exist.

package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.lib.drivers.A21IRSensor;
import com.spartronics4915.lib.drivers.IRSensor;
import com.spartronics4915.lib.util.CANProbe;

import edu.wpi.first.wpilibj.command.Subsystem;
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

            mFrontLeftClimberSolenoid = new DoubleSolenoid(Constants.kClimberPCMId, Constants.kFrontLeftSolenoidId1, Constants.kFrontLeftSolenoidId2);
            mFrontRightClimberSolenoid = new DoubleSolenoid(Constants.kClimberPCMId, Constants.kFrontRightSolenoidId1, Constants.kFrontRightSolenoidId2);
            mRearLeftClimberSolenoid = new DoubleSolenoid(Constants.kClimberPCMId, Constants.kRearLeftSolenoidId1, Constants.kRearLeftSolenoid2);
            mRearRightClimberSolenoid = new DoubleSolenoid(Constants.kClimberPCMId, Constants.kRearRightSolenoidId1, Constants.kRearRightSolenoidId2);
            mClimberFrontIRSensor = new A21IRSensor(Constants.kClimberFrontIRSensorID);
            mClimberRearIRSensor = new A21IRSensor(Constants.kClimberRearIRSensorID);

            success = true;
        }
        catch (Exception e)
        {
            success = false;
            //  logException("Couldn't instantiate hardware: ", e);
        }

        //  logInitialized(success);
    }


    //  Struts will extend from their dormant position to allow the robot to reach the height required to get to L3
    //  Must be done when robot is flushed with L3 (Done with distance sensors and a backup encoder reading)

    public void extendFrontPneumatics()
    {
        mFrontLeftClimberSolenoid.set(Value.kForward);
        mFrontRightClimberSolenoid.set(Value.kForward);
    }

    public void extendBackPneumatics()
    {
        mRearLeftClimberSolenoid.set(Value.kForward);
        mRearRightClimberSolenoid.set(Value.kForward);
    }

    public void retractFrontPneumatics()
    {
        //  Solenoids from the front struts will retract when they become flushed with L3
        //  Done with distance sensors and backup driver vision
        mFrontLeftClimberSolenoid.set(Value.kReverse);
        mFrontRightClimberSolenoid.set(Value.kReverse);
    }

    public void retractBackPneumatics()
    {
        //  Solenoids from the rear struts will retract when the robot can support its own weight on L3
        //  Done primarily with driver vision, but distance sensor might be used
        mRearLeftClimberSolenoid.set(Value.kReverse);
        mRearRightClimberSolenoid.set(Value.kReverse);
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

    @Override
    public void initDefaultCommand()
    {
        //  TODO: add default command (both retracted)
    }

    //  @Override
    public boolean checkSystem(String variant)
    {
        //  logNotice("Lifting for 5 Seconds");
        try
        {
            extendFrontPneumatics();
            extendBackPneumatics();
            Timer.delay(5);
            retractFrontPneumatics();
            retractBackPneumatics();
        }
        catch (Exception e)
        {
            //  logException("Did not pass lifting check: ", e);
            return false;
        }

        //  logNotice("Testing IR Sensors");
        try
        {
            Timer.delay(2);
            mClimberFrontIRSensor.getVoltage();
            //  logNotice("Downward Front IR Sensor Voltage is " + mClimberFrontIRSensor.getVoltage());
            Timer.delay(2);
            mClimberRearIRSensor.getVoltage();
            //  logNotice("Downward Rear IR Sensor Voltage is " + mClimberRearIRSensor.getVoltage());
            Timer.delay(2);
        }
        catch (Exception e)
        {
            //  logException("Did not pass IR sensor check: ", e);
            return false;
        }

        return true;
    }

    //  @Override
    public void outputTelemetry()
    {
        /*
        dashboardPutNumber("Forward sensor voltage: ", mClimberFrontIRSensor.getVoltage());
        dashboardPutBoolean("Forward sensor in range: ", frontSensorsInRange());
        dashboardPutNumber("Rear sensor voltage: ", mClimberRearIRSensor.getVoltage());
        dashboardPutBoolean("Rear sensor in range: ", rearSensorsInRange());
        */
    }
}
