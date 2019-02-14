package com.spartronics4915.frc2019.controlboard;

public interface IButtonControlBoard
{
    // CLIMBING
    boolean getClimb();

    boolean getManualExtendAllClimbPneumatics();

    // INTAKE
    boolean getIntakeCargo(); // vision assisted (?)

    boolean getEjectCargo(); // shared between intake and cargo ramp subsystems

    // CARGO RAMP
    boolean getManualRamp(); // Toggles between MANUAL_RAMP and MANUAL_HOLD

    boolean getShootRocket(); // vision assisted

    boolean getShootBay(); // vision assisted

    boolean getManualShootCargo(); // just ejects the cargo, regardless of the chute height

    boolean getManualToggleChuteHeight(); // toggles the chute height pneumatics

    // PANEL HANDLER
    boolean getIntakePanel();

    boolean getEjectPanel(); // vision assisted

    // EVERYTHING
    boolean getInsideFramePerimeter();
}
