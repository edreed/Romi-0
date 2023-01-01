// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import com.nrg948.preferences.RobotPreferences.EnumValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
@RobotPreferencesLayout(groupName = "Romi", column = 0, row = 0, width = 2, height = 2)
public class RomiStatus {
  public static enum BatteryType {
    /** Using NiMH batteries. */
    NiMH(7.2),
    /** Using Alkaline batteries. */
    Alkaline(9.0);

    private double m_maxVoltage;

    /**
     * Constructs an instance of this enum.
     * 
     * @param maxVoltage The maximum voltage.
     */
    private BatteryType(double maxVoltage) {
      m_maxVoltage = maxVoltage;
    }

    /**
     * Returns the maximum battery voltage.
     * 
     * @return The maximum battery voltage.
     */
    public double getMaxVoltage() {
      return m_maxVoltage;
    }
  }

  /** The type of battery used to power the Romi. */
  @RobotPreferencesValue
  public static final EnumValue<BatteryType> BATTERY_TYPE = new EnumValue<BatteryType>(
      "Romi", "Battery Type", BatteryType.NiMH);

  private static NetworkTable s_rootTable;
  private static NetworkTable s_statusTable;

  /**
   * Returns the Romi root network table.
   * 
   * @return The Romi root network table.
   */
  private static NetworkTable getRootTable() {
    if (s_rootTable == null) {
      s_rootTable = NetworkTableInstance.getDefault().getTable("/Romi");
    }

    return s_rootTable;
  }

  /**
   * Returns the Romi status network table.
   * 
   * @return The Romi status network table.
   */
  private static NetworkTable getStatusTable() {
    if (s_statusTable == null) {
      s_statusTable = getRootTable().getSubTable("Status");
    }

    return s_statusTable;
  }

  /**
   * Returns the maximum battery voltage.
   * 
   * @return The maximum battery voltage.
   */
  public static double getMaxBatteryVoltage() {
    return BATTERY_TYPE.getValue().getMaxVoltage();
  }

  /**
   * Returns the current battery voltage.
   * 
   * @return The current battery voltage.
   */
  public static double getBatteryVoltage() {
    double maxBatteryVoltage = getMaxBatteryVoltage();
    NetworkTableEntry batteryVoltageEntry = getStatusTable().getEntry("Battery Voltage");

    if (batteryVoltageEntry == null) {
      return maxBatteryVoltage;
    }

    return batteryVoltageEntry.getDouble(maxBatteryVoltage);
  }
}
