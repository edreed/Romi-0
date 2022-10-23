// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class RomiStatus {
  // The voltage produced by 6 1.2V NiMH batteries.
  public static final double kMaxVoltage = 7.2;

  private static NetworkTable s_rootTable;
  private static NetworkTable s_statusTable;

  /** Returns the Romi root network table.
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
   * Returns the current battery voltage.
   * 
   * @return The current battery voltage.
   */
  public static double getBatteryVoltage() {
    NetworkTableEntry batteryVoltageEntry = getStatusTable().getEntry("Battery Voltage");

    if (batteryVoltageEntry == null) {
      return kMaxVoltage;
    }

    return batteryVoltageEntry.getDouble(kMaxVoltage);
  }
}
