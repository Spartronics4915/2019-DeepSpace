//  This doesn't work (timing issues) - we never had an autonomous climb.

package com.spartronics4915.frc2019.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class Climb extends CommandGroup
{
    public Climb()
    {
        addSequential(new ClimbExtendAllPneumatics());
        addParallel(new ClimbIntake());
        addSequential(new ClimbRetractFrontPneumatics());
        addSequential(new ClimbRetractBackPneumatics());
    }
}
