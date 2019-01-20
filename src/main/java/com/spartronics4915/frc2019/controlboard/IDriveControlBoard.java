package com.spartronics4915.frc2019.controlboard;

public interface IDriveControlBoard
{

    double getThrottle();

    double getTurn();

    boolean getQuickTurn();

    boolean getReturnToDriverControl();

    boolean getReverseDirection();

    boolean getTestButtonOne();

    boolean getTestButtonTwo();

    boolean getTestButtonThree();
}
