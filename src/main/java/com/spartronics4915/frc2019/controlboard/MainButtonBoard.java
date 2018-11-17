package com.spartronics4915.frc2019.controlboard;

import com.spartronics4915.frc2019.Constants;
import edu.wpi.first.wpilibj.Joystick;

public class MainButtonBoard implements IButtonControlBoard
{

    private static MainButtonBoard mInstance = null;

    public static MainButtonBoard getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new MainButtonBoard();
        }
        return mInstance;
    }

    private final Joystick mButtonBoard = null;

    private MainButtonBoard()
    {
        // mButtonBoard = new Joystick(2); // TODO: Add button board when needed
    }
}
