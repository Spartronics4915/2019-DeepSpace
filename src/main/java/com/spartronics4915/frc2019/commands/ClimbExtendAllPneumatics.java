//  This isn't quite accurate. We had a delay built into the old code to account for weight imbalances.

package com.spartronics4915.frc2019.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ClimbExtendAllPneumatics extends CommandGroup
{
    public ClimbExtendAllPneumatics()
    {
        addParallel(new ClimbExtendFrontPneumatics());
        addParallel(new ClimbExtendBackPneumatics());
    }
}
