// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.nrg948.autonomous.AutonomousCommand;
import com.nrg948.autonomous.AutonomousCommandMethod;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Subsystems;

/** An autonomous command that performs no action. */
@AutonomousCommand(name = "Do Nothing", isDefault = true)
public final class DoNothing extends WaitUntilCommand {

    @AutonomousCommandMethod(name = "Do Nothing (method)")
    public static Command factory(Subsystems subsystems) {
        return new DoNothing(subsystems);
    }

    public DoNothing(Subsystems subsystems) {
        super(() -> !DriverStation.isAutonomousEnabled());
    }
}
