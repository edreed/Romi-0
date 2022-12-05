// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.nrg948.autonomous.Autonomous;
import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveUsingXboxController;
import frc.robot.subsystems.Subsystems;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@RobotPreferencesLayout(groupName = "Preferences", column = 0, row = 2, width = 2, height = 2)
public class RobotContainer {

  // Operator interface object are defined here...
  private final XboxController m_xboxController = new XboxController(0);

  private final JoystickButton m_menuButton = new JoystickButton(m_xboxController, XboxController.Button.kStart.value);

  // The robot's subsystems and commands are defined here...
  private final Subsystems m_subsystems = new Subsystems();

  private final SendableChooser<Command> m_autonomousChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_autonomousChooser = Autonomous.getChooser(m_subsystems, "frc.robot");

    // Configure the button bindings
    configureButtonBindings();

    m_subsystems.drivetrain.setDefaultCommand(
      new DriveUsingXboxController(m_subsystems.drivetrain, m_xboxController));

    initShuffleboard();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_menuButton.whenPressed(() -> m_subsystems.drivetrain.resetPosition());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();
  }

  /** Initializes the Romi Shuffleboard tab. */
  private void initShuffleboard() {
    RobotPreferences.addShuffleBoardTab();
    
    ShuffleboardTab romiTab = Shuffleboard.getTab("Romi");

    ShuffleboardLayout odometryLayout = romiTab.getLayout("Odometry", BuiltInLayouts.kList).
      withPosition(0, 0).
      withSize(2, 3);

    odometryLayout.addNumber("X", () -> m_subsystems.drivetrain.getPose().getX());
    odometryLayout.addNumber("Y", () -> m_subsystems.drivetrain.getPose().getY());
    odometryLayout.add("Heading", new Sendable() {
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", () -> -m_subsystems.drivetrain.getHeading().getDegrees(), null);
      }
    });

    ShuffleboardLayout statusLayout = romiTab.getLayout("Status", BuiltInLayouts.kList)
      .withPosition(2, 0)
      .withSize(2, 3);
    
    statusLayout.addNumber("Battery", () -> RomiStatus.getBatteryVoltage())
      .withWidget(BuiltInWidgets.kVoltageView)
      .withProperties(Map.of("Max", RomiStatus.getMaxBatteryVoltage()));
    
    ShuffleboardLayout autonomousLayout = romiTab.getLayout("Autonomous", BuiltInLayouts.kList)
      .withPosition(4, 0)
      .withSize(2, 3);
    
      autonomousLayout.add("Command", m_autonomousChooser);
  }
}
