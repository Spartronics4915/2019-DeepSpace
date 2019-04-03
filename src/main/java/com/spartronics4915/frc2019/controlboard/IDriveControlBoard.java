package com.spartronics4915.frc2019.controlboard;

public interface IDriveControlBoard
{

    double getThrottle();

    double getTurn();

    boolean getSlowMode();

    boolean getReturnToDriverControl();

    boolean getReverseDirection();

    boolean getDriveToSelectedTarget();

    boolean getTestButtonOne();

    boolean getTestButtonTwo();

    boolean getTestButtonThree();

    //Vision Buttons
    boolean getAssistedIntakeCargo();

    boolean getAssistedShootRocket();

    boolean getAssistedShootBay();

    boolean getAssistedIntakePanel();

    boolean getAssistedEjectPanel();
}
