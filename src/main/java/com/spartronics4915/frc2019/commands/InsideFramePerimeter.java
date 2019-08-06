package com.spartronics4915.frc2019.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class InsideFramePerimeter extends CommandGroup
{
    public InsideFramePerimeter()
    {
        addParallel(new ManualCargoHold());
        //  XXX: Does the chute need to be up to be in the frame perimeter?
        addParallel(new ManualChuteRaise());
    }
}
