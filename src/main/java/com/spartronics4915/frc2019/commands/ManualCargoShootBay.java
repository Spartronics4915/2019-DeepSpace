package com.spartronics4915.frc2019.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ManualCargoShootBay extends CommandGroup
{
    public ManualCargoShootBay()
    {
        addSequential(new ManualChuteRaise());
        addSequential(new ManualCargoShoot());
    }
}
