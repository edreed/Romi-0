// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;

/** An autonomous command that performs no action. */
@AutonomousCommand(name = "Do Nothing", isDefault = true)
public final class DoNothing extends WaitUntilCommand {
    public DoNothing(RobotContainer container) {
        super(() -> !DriverStation.isAutonomousEnabled());
    }
}
