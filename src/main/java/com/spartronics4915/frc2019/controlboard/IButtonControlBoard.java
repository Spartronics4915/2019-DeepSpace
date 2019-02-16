package com.spartronics4915.frc2019.controlboard;

public interface IButtonControlBoard
{
    // CLIMBING
    boolean getClimb();

    boolean getManualExtendAllClimbPneumatics();

    // INTAKE
    boolean getAssistedIntakeCargo(); // vision assisted (?)

    boolean getGroundEjectCargo();

    // CARGO RAMP
    boolean getManualRamp(); // Toggles between MANUAL_RAMP and MANUAL_HOLD

    boolean getAssistedShootRocket(); // vision assisted

    boolean getAssistedShootBay(); // vision assisted
    
    boolean getSelectLeftVisionTarget();

    boolean getSelectRightVisionTarget();

    boolean getManualShootCargoBay();

    boolean getManualShootCargoRocket();

    boolean getManualChuteUp();

    boolean getManualChuteDown();

    // PANEL HANDLER
    boolean getAssistedIntakePanel(); // assisted

    boolean getAssistedEjectPanel(); // vision assisted

    boolean getManualEjectPanel();

    // EVERYTHING
    boolean getInsideFramePerimeter();
}
