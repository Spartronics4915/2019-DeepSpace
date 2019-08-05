package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.lib.util.CANProbe;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.drivers.TalonSRXFactory;
import com.spartronics4915.lib.drivers.A21IRSensor;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CargoChute extends Subsystem
{
    private TalonSRX mRampMotor = null;
    private TalonSRX mRampMotorSlave = null;
    private Solenoid mRampSolenoid = null;
    private A21IRSensor mRampSensor = null;

    public CargoChute()
    {
        boolean success = false;
        try
        {
            if (!CANProbe.getInstance().validatePCMId(Constants.kCargoHatchArmPCMId))
                throw new RuntimeException("CargoChute PCM isn't on the CAN bus!");

            mRampMotor = TalonSRXFactory.createDefaultTalon(Constants.kRampMotorId);
            mRampMotorSlave = TalonSRXFactory.createDefaultTalon(Constants.kRampMotorSlaveId);
            mRampMotorSlave = TalonSRXFactory.createPermanentSlaveTalon(Constants.kRampMotorSlaveId,
                    Constants.kRampMotorId);
            mRampMotorSlave.setInverted(false);

            mRampMotor.setNeutralMode(NeutralMode.Brake);
            mRampMotorSlave.setNeutralMode(NeutralMode.Brake);

            mRampSolenoid = new Solenoid(Constants.kCargoHatchArmPCMId, Constants.kRampSolenoidId);
            mRampSensor = new A21IRSensor(Constants.kRampSensorId);
            success = true;
        }
        catch (Exception e)
        {
            success = false;
            //  logException("Couldn't instantiate hardware", e);
        }

        //  logInitialized(success);
    }

    public void intake()
    {
        mRampMotor.set(ControlMode.PercentOutput, Constants.kRampSpeed);
    }

    public void eject()
    {
        mRampMotor.set(ControlMode.PercentOutput, -Constants.kEjectSpeed);
    }

    public void stop()
    {
        mRampMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void raise()
    {
        mRampSolenoid.set(Constants.kRampSolenoidExtend);
    }

    public void lower()
    {
        mRampSolenoid.set(Constants.kRampSolenoidRetract);
    }

    public boolean ballInPosition()
    {
        return mRampSensor.getVoltage() >= Constants.kMinBallInChuteVoltage;
    }

    @Override
    public void initDefaultCommand()
    {
        //  TODO: add default command (lowered + stopped)
    }

    //  @Override
    public boolean checkSystem(String variant)
    {
        //  logNotice("Beginning CargoChute system check:");

        //  logNotice("Beginning ramp check.");
        try
        {
            //  logNotice("Running RampMotor at ramping speed for five seconds: ");
            intake();
            Timer.delay(5);
            //  logNotice("Done.");

            //  logNotice("Running RampMotor at zero speed for three seconds: ");
            stop();
            Timer.delay(3);
            //  logNotice("Done.");

            //  logNotice("Running RampMotor at reverse ramping speed for five seconds: ");
            eject();
            Timer.delay(5);
            //  logNotice("Done.");

            stop();
        }
        catch (Exception e)
        {
            //  logException("Did not pass ramp check: ", e);
            return false;
        }
        //  logNotice("Ramp check complete.");

        //  logNotice("Beginning pneumatic check: ");
        try
        {
            //  logNotice("Sending ramp pneumatics up for three seconds: ");
            raise();
            Timer.delay(3);
            //  logNotice("Done.");

            //  logNotice("Sending ramp pneumatics down for three seconds: ");
            lower();
            Timer.delay(3);
            //  logNotice("Done.");

            //  logNotice("Sending ramp pneumatics up for three seconds: ");
            raise();
            Timer.delay(3);
            //  logNotice("Done.");

            //  logNotice("Sending ramp pneumatics down for three seconds: ");
            lower();
            Timer.delay(3);
            //  logNotice("Done.");
        }
        catch (Exception e)
        {
            //  logException("Did not pass pneumatic check: ", e);
            return false;
        }
        //  logNotice("Pneumatic check complete.");

        //  logNotice("CargoChute system check complete.");
        return true;
    }

    //  @Override
    public void outputTelemetry()
    {
        /*
        dashboardPutBoolean("mRampSolenoid extended: ", mRampSolenoid.get());
        dashboardPutNumber("mRampMotor speed: ", mRampMotor.getMotorOutputPercent());
        dashboardPutNumber("mRampSensor voltage: ", mRampSensor.getVoltage());
        dashboardPutBoolean("Ball in position: ", ballInPosition());
        */
    }
}
