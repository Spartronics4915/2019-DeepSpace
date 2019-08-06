package com.spartronics4915.frc2019.commands;

import com.spartronics4915.frc2019.subsystems.CargoChute;
import com.spartronics4915.frc2019.subsystems.CargoIntake;

import edu.wpi.first.wpilibj.command.Command;

public class ManualHold extends Command
{
    private CargoChute mCargoChute;
    private CargoIntake mCargoIntake;

    public ManualHold()
    {
        mCargoChute = CargoChute.getInstance();
        mCargoIntake = CargoIntake.getInstance();
        //  Use requires() here to declare subsystem dependencies
        requires(mCargoChute);
        requires(mCargoIntake);
    }

    //  Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        setInterruptible(false);

        if (!mCargoIntake.isArmClimb())
        {
            mCargoIntake.stop();
            if (mCargoIntake.isArmDown())
                mCargoIntake.armUp();
            mCargoChute.stop();
        }
    }

    //  Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        //  Intentionally left blank
    }

    //  Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished()
    {
        return true;
    }

    //  Called once after isFinished returns true
    @Override
    protected void end()
    {
    }

    //  Called when another command which requires one or more of the same
    //  subsystems is scheduled to run
    @Override
    protected void interrupted()
    {
    }
}
