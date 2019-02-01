package com.spartronics4915.frc2019.controlboard;

import com.spartronics4915.frc2019.Constants;
import edu.wpi.first.wpilibj.Joystick;

public class MainButtonBoard implements IButtonControlBoard
{
    private final Joystick mButtonBoard;

    public MainButtonBoard()
    {
        mButtonBoard = new Joystick(Constants.kMainButtonBoardPort);
    }


    //TODO: assign these buttons

    @Override
    public boolean getClimb()
    {
        return mButtonBoard.getRawButtonPressed(1);
    }

    @Override
    public boolean getIntake()
    {
        return mButtonBoard.getRawButtonPressed(2);
    }

    @Override
    public boolean getEjectCargo()
    {
        return mButtonBoard.getRawButtonPressed(3);
    }

    @Override
    public boolean getShootRocket()
    {
        return mButtonBoard.getRawButtonPressed(4);
    }

    @Override
    public boolean getShootBay()
    {
        return mButtonBoard.getRawButtonPressed(5);
    }

    @Override
    public boolean getEjectPanel()
    {
        return mButtonBoard.getRawButtonPressed(6);
    }
}
