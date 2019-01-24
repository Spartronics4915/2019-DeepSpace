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
            new AutoModeCreator("Other: Straight Path Test", () -> new PathTestMode(false)),
            new AutoModeCreator("Other: Curved Path Test", () -> new PathTestMode(true)),
            new AutoModeCreator("Other: Velocity Test", () -> new VelocityTestMode()),
            new AutoModeCreator("Other: Characterize Drivetrain (Both)", () -> new CharacterizeDriveMode(SideToCharacterize.BOTH)),
            new AutoModeCreator("Other: Characterize Drivetrain (Left)", () -> new CharacterizeDriveMode(SideToCharacterize.LEFT)),
            new AutoModeCreator("Other: Characterize Drivetrain (Right)", () -> new CharacterizeDriveMode(SideToCharacterize.RIGHT)),

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
