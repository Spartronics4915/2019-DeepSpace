package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.commands.PanelRetract;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import com.spartronics4915.lib.hardware.motors.CANProbe;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
//  import edu.wpi.first.wpilibj.DigitalInput;

/**
 * 2 pneumatics to eject panels
 * panels held on by velcro
 */

public class PanelHandler extends SpartronicsSubsystem
{
    private static PanelHandler mInstance = null;

    public static PanelHandler getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new PanelHandler();
        }
        return mInstance;
    }

    private Solenoid mSolenoid = null;
    //  private DigitalInput mLimitSwitch = null;

    private PanelHandler()
    {
        boolean success = false;
        try
        {
            if (!CANProbe.getInstance().validatePCMId(Constants.kCargoHatchArmPCMId)) throw new RuntimeException("PanelHandler PCM isn't on the CAN bus!");

            mSolenoid = new Solenoid(Constants.kCargoHatchArmPCMId, Constants.kPanelHandlerSolenoid);
            //  mLimitSwitch
            success = true;
        }
        catch (Exception e)
        {
            success = false;
            logException("Couldn't instantiate hardware: ", e);
        }

        logInitialized(success);
    }

    public void extend()
    {
        mSolenoid.set(Constants.kPanelSolenoidExtend);

    }

    public void retract()
    {
        mSolenoid.set(Constants.kPanelSolenoidRetract);
    }

    @Override
    public void initDefaultCommand()
    {
        setDefaultCommand(new PanelRetract());
    }

    public boolean checkSystem(String variant)
    {
        logNotice("Starting PanelHandler Solenoid Check");
        try
        {
            logNotice("Extending solenoid for 2 seconds");
            extend();
            Timer.delay(2);
            logNotice("Retracting solenoid for 2 seconds");
            retract();
            Timer.delay(2);
            logNotice("Extending solenoid for 2 seconds");
            extend();
            Timer.delay(2);
            logNotice("Retracting solenoid for 2 seconds");
            retract();
        }
        catch (Exception e)
        {
            logException("Trouble instantiating hardware ", e);
            return false;
        }
        logNotice("PanelHandler Solenoid Check End");
        return true;
    }

    @Override
    public void outputTelemetry()
    {
        dashboardPutBoolean("mSolenoid1 Extended: ", mSolenoid.get());
        //  dashboardPutBoolean("Is a Panel aquired?", mLimitSwitch.get());
    }
}
