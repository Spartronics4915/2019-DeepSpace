/* TODO
 *
 * Change motor names to align with the front of the robot
 */

package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.commands.CargoManualHold;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRXFactory;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import com.spartronics4915.lib.hardware.motors.CANProbe;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class CargoIntake extends SpartronicsSubsystem
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

    private Solenoid mSolenoid = null;
    private Solenoid mSolenoidClimb = null;
    private TalonSRX mMotorRight = null;    //  right and left are switched?
    private TalonSRX mMotorLeft = null;

    private CargoIntake()
    {
        boolean success = false;    //  IR sensor analog port 6 to detect cargo going into chute. Used by chute as well.
        try
        {
            if (!CANProbe.getInstance().validatePCMId(Constants.kCargoHatchArmPCMId))
                throw new RuntimeException("CargoIntake PCM isn't on the CAN bus!");

            mMotorRight = SpartronicsSRXFactory.createDefaultTalon(Constants.kCargoIntakeMotorRight);
            mMotorLeft = SpartronicsSRXFactory.createDefaultTalon(Constants.kCargoIntakeMotorLeft);

            mMotorRight.setNeutralMode(NeutralMode.Brake);
            mMotorLeft.setNeutralMode(NeutralMode.Brake);

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

    public void intake()
    {
        mMotorRight.set(ControlMode.PercentOutput, Constants.kCargoIntakeSpeed);
        mMotorLeft.set(ControlMode.PercentOutput, Constants.kCargoIntakeSpeed);
    }

    public void eject()
    {
        mMotorRight.set(ControlMode.PercentOutput, Constants.kCargoEjectSpeed);
        mMotorLeft.set(ControlMode.PercentOutput, Constants.kCargoEjectSpeed);
    }

    public void stop()
    {
        mMotorRight.set(ControlMode.PercentOutput, 0.0);
        mMotorLeft.set(ControlMode.PercentOutput, 0.0);
    }

    public void armUp()
    {
        mSolenoid.set(Constants.kCargoIntakeSolenoidRetract);
        mSolenoidClimb.set(Constants.kCargoIntakeSolenoidRetract);
    }

    public void armDown()
    {
        mSolenoid.set(Constants.kCargoIntakeSolenoidExtend);
        mSolenoidClimb.set(Constants.kCargoIntakeSolenoidRetract);
    }

    public void armClimb()
    {
        mSolenoid.set(Constants.kCargoIntakeSolenoidExtend);
        mSolenoidClimb.set(Constants.kCargoIntakeSolenoidExtend);
    }

    public boolean isArmDown()
    {
        //  Futureproof
        return (mSolenoid.get() == Constants.kCargoIntakeSolenoidRetract);
    }

    public boolean isArmClimb()
    {
        return (mSolenoidClimb.get() == Constants.kCargoIntakeSolenoidExtend);
    }

    @Override
    public void initDefaultCommand()
    {
        setDefaultCommand(new CargoManualHold());
    }

    public boolean checkSystem(String variant)
    {
        logNotice("Starting CargoIntake Solenoid Check");
        try
        {
            logNotice("Extending solenoids for 4 seconds");
            armDown();
            Timer.delay(4);

            logNotice("Retracting solenoids for 4 seconds");
            armUp();
            Timer.delay(4);

            logNotice("CargoIntake Solenoid Check End");
            logNotice("Running motors at 50% for 2 seconds");
            intake();
            Timer.delay(2);

            logNotice("Running motors at 0% for 2 seconds");
            stop();
            Timer.delay(2);

            logNotice("Running motors at -50% for 2 seconds");
            eject();
            Timer.delay(2);

            stop();
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
        dashboardPutBoolean("mSolenoid Extended", mSolenoid.get());
        dashboardPutBoolean("mSolenoidClimb Extended", mSolenoidClimb.get());
        dashboardPutNumber("mMotor1 Speed", mMotorRight.getMotorOutputPercent());
        dashboardPutNumber("mMotor2 Speed", mMotorLeft.getMotorOutputPercent());
    }
}
