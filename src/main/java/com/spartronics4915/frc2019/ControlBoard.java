package com.spartronics4915.frc2019;

import com.spartronics4915.frc2019.controlboard.*;

public class ControlBoard implements IControlBoard
{

    private static ControlBoard mInstance = null;

    public static ControlBoard getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    private IDriveControlBoard mDriveControlBoard;
    private IButtonControlBoard mButtonControlBoard;

    private ControlBoard()
    {
        // Switch between control boards here

        mDriveControlBoard = MainDriveControlBoard.getInstance();

        mButtonControlBoard = MainButtonBoard.getInstance();
    }

    @Override
    public double getThrottle()
    {
        return mDriveControlBoard.getThrottle();
    }

    @Override
    public double getTurn()
    {
        return mDriveControlBoard.getTurn();
    }

    @Override
    public boolean getQuickTurn()
    {
        return mDriveControlBoard.getQuickTurn();
    }

    @Override
    public boolean getReturnToDriverControl()
    {
        return mDriveControlBoard.getReturnToDriverControl();
    }

    @Override
    public boolean getReverseDirection()
    {
        return mDriveControlBoard.getReverseDirection();
    }

    @Override
    public boolean getDriveToSelectedTarget()
    {
        return mDriveControlBoard.getDriveToSelectedTarget();
    }

    @Override
    public boolean getTestButtonOne() {
        return mDriveControlBoard.getTestButtonOne();
    }

    @Override
    public boolean getTestButtonTwo() {
        return mDriveControlBoard.getTestButtonTwo();
    }

    @Override
    public boolean getTestButtonThree() {
        return mDriveControlBoard.getTestButtonThree();
    }
}
