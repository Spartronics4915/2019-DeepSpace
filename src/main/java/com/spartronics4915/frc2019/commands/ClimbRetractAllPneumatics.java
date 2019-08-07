//  This is a CommandGroup because it is only a default command. Timing doesn't matter.

package com.spartronics4915.frc2019.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ClimbRetractAllPneumatics extends CommandGroup
{
    public ClimbRetractAllPneumatics()
    {
        addParallel(new ClimbRetractBackPneumatics());
        addParallel(new ClimbRetractFrontPneumatics());
    }
}
