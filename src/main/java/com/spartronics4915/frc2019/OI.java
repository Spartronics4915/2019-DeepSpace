package com.spartronics4915.frc2019;

import com.spartronics4915.frc2019.commands.*;
import com.spartronics4915.frc2019.Constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI
{
    //  NOTE: I'm not defining ints.
    public static final Joystick sDriveStick = new Joystick (Constants.kDriveJoystickPort);
    public static final Joystick sArcadeStick = new Joystick(Constants.kMainButtonBoardPort);

    public OI()
    {
        /**
         *  Nomenclature is different between Commands and JoystickButtons.
         *  Commands highlight if it is a manual override, what element of the robot it uses (i.e. climb),
         *  then a description of its action.
         *  JoystickButtons highlight the element of robot and then a description.
         *
         *  Several "manual" buttons have not had the classification carry over.
         *  This is because the vision-capable versions of them would completely replace
         *  the manuals, once implemented.
         *  Distinctions are still made for manual overrides of IR sensors and ilk.
         */

        //  NOTE: The button assignments are slightly different

        //  Arcade Stick buttons

        JoystickButton cargoIntakeArcadeStick = new JoystickButton(sArcadeStick, 1);

        JoystickButton panelEjectArcadeStick = new JoystickButton(sArcadeStick, 2);

        JoystickButton cargoManualHoldArcadeStick = new JoystickButton(sArcadeStick, 3);
        JoystickButton cargoEjectArcadeStick = new JoystickButton(sArcadeStick, 4);

        JoystickButton cargoShootRocketArcadeStick = new JoystickButton(sArcadeStick, 5);
        JoystickButton cargoShootBayArcadeStick = new JoystickButton(sArcadeStick, 6);

        JoystickButton cargoManualIntakeArcadeStick = new JoystickButton(sArcadeStick, );   //  Left on the d-stick

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
        JoystickButton visionSelectTargetDriveStick = new JoystickButton(sDriveStick, 5);

        JoystickButton assistedShootBayDriveStick = new JoystickButton(sDriveStick, 6);
        JoystickButton assistedShootRocketDriveStick = new JoystickButton(sDriveStick, 7);

        JoystickButton insideFramePerimeterDriveStick = new JoystickButton(sDriveStick, 8);
        JoystickButton insideFramePerimeterTwoDriveStick = new JoystickButton(sDriveStick, 9);

        JoystickButton panelAssistedEjectDriveStick = new JoystickButton(sDriveStick, 10);
        JoystickButton panelAssistedIntakeDriveStick = new JoystickButton(sDriveStick, 11);


        //  Arcade Stick commands
        cargoIntakeArcadeStick.whenPressed(new CargoIntakeCommand());

        panelEjectArcadeStick.whenPressed(new ManualPanelEject());

        cargoManualHoldArcadeStick.whenPressed(new ManualCargoHold());
        cargoEjectArcadeStick.whenPressed(new CargoEject());

        cargoShootRocketArcadeStick.whenPressed(new ManualCargoShootRocket());
        cargoShootBayArcadeStick.whenPressed(new ManualCargoShootBay());

        cargoManualIntakeArcadeStick.whenPressed(new ManualCargoIntake());

        chuteRaiseArcadeStick.whenPressed(new ManualChuteRaise());
        chuteLowerArcadeStick.whenPressed(new ManualChuteLower());

        climbExtendAllPneumaticsArcadeStick.whenPressed(new ClimbExtendAllPneumatics());
        climbIntakeDownArcadeStick.whenPressed(new ClimbIntake());
        climbRetractFrontPneumaticsArcadeStick.whenPressed(new ClimbRetractFrontPneumatics());
        climbRetractBackPneumaticsArcadeStick.whenPressed(new ClimbRetractBackPneumatics());

        //  Drive Stick commands
        insideFramePerimeterDriveStick.whenPressed(new InsideFramePerimeter());
        insideFramePerimeterTwoDriveStick.whenPressed(new InsideFramePerimeter());

    }
}
