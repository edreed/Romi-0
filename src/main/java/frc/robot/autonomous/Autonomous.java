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
   * @param <T> The container type.
   * @param container An object passed to the constructor of the automonous
   *                  commands providing access to the robot subsystems. This
   *                  is typically an instance of {@link RobotContainer} but
   *                  could be another type that manages the subsystems.
   * 
   * @return A {@link SendableChooser} object containing the autonomous commands.
   */
  public static <T> SendableChooser<Command> getChooser(T container) {
    Reflections reflections = new Reflections("frc.robot");
    SendableChooser<Command> chooser = new SendableChooser<>();

    reflections.get(TypesAnnotated.with(AutonomousCommand.class).asClass())
        .stream()
        .forEach(cc -> {
          try {
            AutonomousCommand annotation = cc.getAnnotation(AutonomousCommand.class);
            Command command = (Command) cc.getConstructor(container.getClass()).newInstance(container);

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
