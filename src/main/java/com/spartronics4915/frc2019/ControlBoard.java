package com.spartronics4915.frc2019;

import com.spartronics4915.frc2019.controlboard.*;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.GenericHID.HIDType;
import edu.wpi.first.wpilibj.Joystick;

public class ControlBoard implements IControlBoard
{

    private IDriveControlBoard mDriveControlBoard;
    private IButtonControlBoard mButtonControlBoard;

    private ControlBoard()
    {
        // Possibly use Joystick::getType instead of Joystick::getName
        String joyName = new Joystick(Constants.kDriveJoystickPort).getName();
        switch (joyName)
        {
            case "Xbox": // TODO: Figure out these names
                mDriveControlBoard = new XboxSplitControlBoard();
                break;
            case "Joystick":
                if (new Joystick(3).getType() != HIDType.kUnknown)
                    mDriveControlBoard = new TwoJoystickSplitControlBoard();
                else
                    mDriveControlBoard = new OneJoystickControlBoard();
                break;
        }
        Logger.debug("Found joystick " + joyName + " on port 0, selected IControlBoard implementer " + mDriveControlBoard.getClass().getName());

        mButtonControlBoard = new MainButtonBoard();
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
    public boolean getTestButtonOne()
    {
        return mDriveControlBoard.getTestButtonOne();
    }

    @Override
    public boolean getTestButtonTwo()
    {
        return mDriveControlBoard.getTestButtonTwo();
    }

    @Override
    public boolean getTestButtonThree()
    {
        return mDriveControlBoard.getTestButtonThree();
    }
}
