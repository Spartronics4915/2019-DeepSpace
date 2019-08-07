package com.spartronics4915.frc2019.commands;

import com.spartronics4915.frc2019.subsystems.CargoChute;
import com.spartronics4915.frc2019.subsystems.CargoIntake;

import edu.wpi.first.wpilibj.command.Command;

public class CargoManualIntake extends Command
{
    private CargoChute mCargoChute;
    private CargoIntake mCargoIntake;

    public CargoManualIntake()
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
        setInterruptible(true);

        //  XXX: Does this cancel the command?
        if (mCargoIntake.isArmClimb())
            return;

        //  Put the arm down if not already down
        if (!mCargoIntake.isArmDown())
            mCargoIntake.armDown();
    }

    //  Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        mCargoIntake.intake();
        mCargoChute.intake();
    }

    //  Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished()
    {
        //  This command executes until interrupted.
        return false;
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
        mCargoIntake.stop();
        mCargoIntake.armUp();
        mCargoChute.stop();
    }
}
