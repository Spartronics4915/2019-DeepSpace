package com.spartronics4915.frc2019.controlboard;

public interface IButtonControlBoard
{
    // CLIMBING
    boolean getClimb();

    boolean getManualExtendAllClimbPneumatics();

    // INTAKE
    boolean getIntakeCargo();

    boolean getEjectCargo(); // shared between intake and cargo ramp subsystems

    // CARGO RAMP
    boolean getManualRamp(); // Toggles between MANUAL_RAMP and MANUAL_HOLD

    boolean getShootRocket();

    boolean getShootBay();

    // PANEL HANDLER
    boolean getIntakePanel();

    boolean getEjectPanel();
}
