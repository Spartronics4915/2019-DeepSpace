package com.spartronics4915.frc2019.controlboard;

public interface IButtonControlBoard
{
    void updatePOV();
    
    // CLIMBING
    boolean getClimb();

    boolean getClimbExtendAllPneumatics();

    boolean getClimbIntake();

    boolean getClimbRetractFrontPneumatics();

    boolean getClimbRetractBackPneumatics();

    // INTAKE
    boolean getAssistedIntakeCargo(); // vision assisted (?)

    boolean getGroundEjectCargo();

    boolean getManualIntakeCargo();

    boolean getIntakeArmDown();

    boolean getIntakeHold();

    boolean getIntakeStopMotors();

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

    boolean getChangeSelectedVisionIndex();
}
