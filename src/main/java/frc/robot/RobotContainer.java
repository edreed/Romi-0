// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveUsingXboxController;
import frc.robot.subsystems.RSL;
import frc.robot.subsystems.RomiDrivetrain;
import java.util.Map;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Operator interface object are defined here...
  private final XboxController m_xboxController = new XboxController(0);

  // The robot's subsystems and commands are defined here...
  @SuppressWarnings("unused")
  private final RSL m_rsl = new RSL();
  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();

  private final Command m_autoCommand = new WaitCommand(1.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_romiDrivetrain.setDefaultCommand(new DriveUsingXboxController(m_romiDrivetrain, m_xboxController));

    initShuffleboard();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  /** Initializes the Romi Shuffleboard tab. */
  private void initShuffleboard() {
    ShuffleboardTab romiTab = Shuffleboard.getTab("Romi");

    ShuffleboardLayout odometryLayout = romiTab.getLayout("Odometry", BuiltInLayouts.kList).
      withPosition(0, 0).
      withSize(2, 3);

    odometryLayout.addNumber("Y", () -> m_romiDrivetrain.getPose().getY());
    odometryLayout.addNumber("X", () -> m_romiDrivetrain.getPose().getX());
    odometryLayout.add("Heading", new Sendable() {
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", () -> m_romiDrivetrain.getHeading().getDegrees(), null);
      }
    });

    ShuffleboardLayout statusLayout = romiTab.getLayout("Status", BuiltInLayouts.kList).
      withPosition(2, 0).
      withSize(2, 3);
    
    statusLayout.addNumber("Battery", () -> RobotController.getBatteryVoltage()).
      withWidget(BuiltInWidgets.kVoltageView).
      withProperties(Map.of("Max", 7.2));
  }
}
