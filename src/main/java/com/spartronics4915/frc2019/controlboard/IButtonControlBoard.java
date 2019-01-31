package com.spartronics4915.frc2019.controlboard;

public interface IButtonControlBoard
{
    //CLIMBING
    boolean getClimb();

    //INTAKE
    boolean getIntake();

    boolean getEjectCargo(); //intake and cargo ramp subsystems

    //CARGO RAMP
    boolean getShootRocket();

    boolean getShootBay();

    //PANEL HANDLER
    boolean getEjectPanel();
}
