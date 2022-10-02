// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A command that enables driving the robot using an Xbox controller. */
public class DriveUsingXboxController extends CommandBase {
  private final RomiDrivetrain m_romiDrivetrain;
  private final XboxController m_xboxController;
  private final StringLogEntry m_commandStateLogger = new StringLogEntry(DataLogManager.getLog(), "/command/DriveUsingXboxController/state");

  /**
   * Creates a new DriveUsingXboxController.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveUsingXboxController(RomiDrivetrain subsystem, XboxController xboxController) {
    m_romiDrivetrain = subsystem;
    m_xboxController = xboxController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_commandStateLogger.append("initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -MathUtil.applyDeadband(m_xboxController.getLeftY(), 0.05);
    double rotation = MathUtil.applyDeadband(m_xboxController.getRightX(), 0.05);
 
    m_romiDrivetrain.arcadeDrive(speed, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_romiDrivetrain.stopMotor();
    m_commandStateLogger.append("end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
