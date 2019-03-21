package com.spartronics4915.frc2019;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.modes.*;
import com.spartronics4915.frc2019.auto.modes.CharacterizeDriveMode.SideToCharacterize;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class that allows a user to select which autonomous mode to execute from the
 * web dashboard.
 */
public class AutoModeSelector
{

    public static final String AUTO_OPTIONS_DASHBOARD_KEY = "AutoStrategyOptions";
    public static final String SELECTED_AUTO_MODE_DASHBOARD_KEY = "AutoStrategy";

    private static class AutoModeCreator
    {

        private final String mDashboardName;
        private final Supplier<AutoModeBase> mCreator;

        private AutoModeCreator(String dashboardName, Supplier<AutoModeBase> creator)
        {
            mDashboardName = dashboardName;
            mCreator = creator;
        }
    }

    private static final AutoModeCreator mDefaultMode =
            new AutoModeCreator("All: Do nothing", () -> new DoNothingMode());
    private static final AutoModeCreator[] mAllModes = {
            mDefaultMode,
            new AutoModeCreator("Left: Drive Off Hab and Place Parallel Panel", () -> new PlaceHatchFromSideMode(true)),
            new AutoModeCreator("Right: Drive Off Hab and Place Parallel Panel", () -> new PlaceHatchFromSideMode(false)),
            new AutoModeCreator("Left: Drive Off Hab and Place Close Left Cargo", () -> new PlaceCargoFromSideMode(true)),
            new AutoModeCreator("Right: Drive Off Hab and Place Close Left Cargo", () -> new PlaceCargoFromSideMode(false)),

            new AutoModeCreator("Middle: Drive Off Middle of Hab and Place Left Panel", ()-> new PlaceHatchFromMiddleMode(true)),
            new AutoModeCreator("Middle: Drive Off Middle of Hab and Place Right Panel", ()-> new PlaceHatchFromMiddleMode(false)),
            new AutoModeCreator("Middle: Drive Off Middle of Hab and Place Left Close Bay", () -> new PlaceCargoFromMiddleMode(true)),
            new AutoModeCreator("Middle: Drive Off Middle of Hab and Palce Right Close Bay", () -> new PlaceCargoFromMiddleMode(false)),
            
            
            new AutoModeCreator("Other: Drive Back to Depot Test (Left)", () -> new DriveToDepotTestMode(true)),
            new AutoModeCreator("Other: Drive Back to Depot Test (Right)", () -> new DriveToDepotTestMode(false)),
            new AutoModeCreator("Other: Drive Off HAB Test", () -> new DriveOffHABTestMode()),
            new AutoModeCreator("Other: Straight Path Test", () -> new PathTestMode(false)),
            new AutoModeCreator("Other: Curved Path Test", () -> new PathTestMode(true)),
            new AutoModeCreator("Other: Velocity Test", () -> new VelocityTestMode()),
            new AutoModeCreator("Other: Diagnose Dropouts", () -> new DiagnoseDropoutsMode()),
            new AutoModeCreator("Other: Characterize Drivetrain (Both+MOI)", () -> new CharacterizeDriveMode(SideToCharacterize.BOTH)),
            new AutoModeCreator("Other: Characterize Drivetrain (Left)", () -> new CharacterizeDriveMode(SideToCharacterize.LEFT)),
            new AutoModeCreator("Other: Characterize Drivetrain (Right)", () -> new CharacterizeDriveMode(SideToCharacterize.RIGHT)),
            new AutoModeCreator("Other: Characterize Drivetrain (Remote)", () -> new CharacterizeDriveRemoteMode()),
            new AutoModeCreator("Other: Find Effective Wheelbase Diameter", () -> new FindEffectiveWheelbaseDiameter())

            // e.g. new AutoModeCreator(C: Drive To Hopper", () -> new DriveToHopperMode()),
    };

    public static void updateSmartDashboard()
    {
        Set<String> modesArray = new HashSet<>();
        for (AutoModeCreator mode : mAllModes)
        {
            modesArray.add(mode.mDashboardName);
        }
        SmartDashboard.putString(AUTO_OPTIONS_DASHBOARD_KEY, String.join(",", modesArray));
    }

    public static AutoModeBase getSelectedAutoMode()
    {
        String selectedModeName = SmartDashboard.getString(SELECTED_AUTO_MODE_DASHBOARD_KEY, "NO SELECTED MODE!!!!");
        Logger.notice("Auto mode name " + selectedModeName);
        for (AutoModeCreator mode : mAllModes)
        {
            if (mode.mDashboardName.equals(selectedModeName))
            {
                return mode.mCreator.get();
            }
        }
        Logger.error("AutoModeSelector failed to select auto mode: " + selectedModeName);
        return mDefaultMode.mCreator.get();
    }
}
