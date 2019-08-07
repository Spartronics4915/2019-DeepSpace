package com.spartronics4915.frc2019;

import com.spartronics4915.frc2019.commands.*;
import com.spartronics4915.frc2019.Constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI
{
    //  NOTE: I'm not defining ints.
    public static final Joystick sDriveStick = new Joystick (Constants.kDriveJoystickPort);
    public static final Joystick sArcadeStick = new Joystick(Constants.kMainButtonBoardPort);

    public OI()
    {
        //  NOTE: The button assignments are slightly different

        //  Arcade Stick buttons
        JoystickButton cargoIRIntakeArcadeStick = new JoystickButton(sArcadeStick, 1);

        JoystickButton panelEjectArcadeStick = new JoystickButton(sArcadeStick, 2);

        JoystickButton cargoManualHoldArcadeStick = new JoystickButton(sArcadeStick, 3);
        JoystickButton cargoEjectArcadeStick = new JoystickButton(sArcadeStick, 4);

        JoystickButton cargoShootRocketArcadeStick = new JoystickButton(sArcadeStick, 5);
        JoystickButton cargoShootBayArcadeStick = new JoystickButton(sArcadeStick, 6);

        JoystickButton cargoManualIntakeArcadeStick = new JoystickButton(sArcadeStick, );   //  Left on the d-stick
        //  TODO: Add d-stick inputs
        JoystickButton chuteRaiseArcadeStick = new JoystickButton(sArcadeStick, );  //  Up on the d-stick
        JoystickButton chuteLowerArcadeStick = new JoystickButton(sArcadeStick, );  //  Down on the d-stick

        JoystickButton climbExtendAllPneumaticsArcadeStick = new JoystickButton(sArcadeStick, 7);
        JoystickButton climbIntakeDownArcadeStick = new JoystickButton(sArcadeStick, 8);
        JoystickButton climbRetractFrontPneumaticsArcadeStick = new JoystickButton(sArcadeStick, 9);
        JoystickButton climbRetractBackPneumaticsArcadeStick = new JoystickButton(sArcadeStick, 10);

        //  Drive Stick buttons
        JoystickButton driveSlowModeDriveStick = new JoystickButton(sDriveStick, 1);
        JoystickButton driveReturnToDriverControlDriveStick = new JoystickButton(sDriveStick, 2);
        JoystickButton driveReverseDirectionDriveStick = new JoystickButton(sDriveStick, 3);
        JoystickButton driveToSelectedTargetDriveStick = new JoystickButton(sDriveStick, 4);
        JoystickButton driveChangeSelectedTargetDriveStick = new JoystickButton(sDriveStick, 5);   //  Cycles through vision targets - not enough buttons for left/right

        JoystickButton cargoAssistedShootBayDriveStick = new JoystickButton(sDriveStick, 6);
        JoystickButton cargoAssistedShootRocketDriveStick = new JoystickButton(sDriveStick, 7);
        JoystickButton panelAssistedEjectDriveStick = new JoystickButton(sDriveStick, 10);
        JoystickButton panelAssistedIntakeDriveStick = new JoystickButton(sDriveStick, 11);

        JoystickButton insideFramePerimeterDriveStick = new JoystickButton(sDriveStick, 8);
        JoystickButton insideFramePerimeterTwoDriveStick = new JoystickButton(sDriveStick, 9);

        //  TODO: Actual drive stick???

        //  Arcade Stick commands
        cargoIRIntakeArcadeStick.whenPressed(new CargoIRIntake());

        panelEjectArcadeStick.whenPressed(new PanelEject());

        cargoManualHoldArcadeStick.whenPressed(new CargoManualHold());
        cargoEjectArcadeStick.whenPressed(new CargoEject());

        cargoShootRocketArcadeStick.whenPressed(new CargoShootRocket());
        cargoShootBayArcadeStick.whenPressed(new CargoShootBay());

        cargoManualIntakeArcadeStick.whenPressed(new CargoManualIntake());

        chuteRaiseArcadeStick.whenPressed(new ChuteRaise());
        chuteLowerArcadeStick.whenPressed(new ChuteLower());

        climbExtendAllPneumaticsArcadeStick.whenPressed(new ClimbExtendAllPneumatics());
        climbIntakeDownArcadeStick.whenPressed(new ClimbIntake());
        climbRetractFrontPneumaticsArcadeStick.whenPressed(new ClimbRetractFrontPneumatics());
        climbRetractBackPneumaticsArcadeStick.whenPressed(new ClimbRetractBackPneumatics());

        //  Drive Stick commands
        /*
        driveSlowModeDriveStick.whileHeld();
        driveReturnToDriverControlDriveStick.whenPressed();
        driveReverseDirectionDriveStick.whenPressed();
        driveToSelectedTargetDriveStick.whenPressed();
        driveChangeSelectedTargetDriveStick.whenPressed();

        cargoAssistedShootBayDriveStick.whenPressed();
        cargoAssistedShootRocketDriveStick.whenPressed();
        panelAssistedEjectDriveStick.whenPressed();
        panelAssistedIntakeDriveStick.whenPressed();
        */

        insideFramePerimeterDriveStick.whenPressed(new InsideFramePerimeter());
        insideFramePerimeterTwoDriveStick.whenPressed(new InsideFramePerimeter());

    }
}
