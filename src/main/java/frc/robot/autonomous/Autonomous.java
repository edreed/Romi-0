// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import static org.reflections.scanners.Scanners.TypesAnnotated;

import java.lang.reflect.InvocationTargetException;

import org.reflections.Reflections;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/** A class containing utility methods to support autonomous operation. */
public final class Autonomous {

  /**
   * Returns a {@link SendableChooser} object enabling interactive selection of
   * autonomous commands annotated with {@link AutonomousCommand}.
   * 
   * @param container The {@link RobotContainer} instance of the robot.
   * 
   * @return A {@link SendableChooser} object containing the autonomous commands.
   */
  public static SendableChooser<Command> getChooser(RobotContainer container) {
    Reflections reflections = new Reflections("frc.robot");
    SendableChooser<Command> chooser = new SendableChooser<>();

    reflections.get(TypesAnnotated.with(AutonomousCommand.class).asClass())
        .stream()
        .forEach(cc -> {
          try {
            AutonomousCommand annotation = cc.getAnnotation(AutonomousCommand.class);
            Command command = (Command) cc.getConstructor(RobotContainer.class).newInstance(container);

            chooser.addOption(annotation.name(), command);

            if (annotation.isDefault()) {
              chooser.setDefaultOption(annotation.name(), command);
            }
          } catch (
              InstantiationException
              | IllegalAccessException
              | IllegalArgumentException
              | ClassCastException
              | InvocationTargetException
              | NoSuchMethodException
              | SecurityException e) {
            e.printStackTrace();
          }
        });

    return chooser;
  }

  private Autonomous() {

  }
}
