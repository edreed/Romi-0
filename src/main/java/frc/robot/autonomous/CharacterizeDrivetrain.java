// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.nrg948.autonomous.AutonomousCommand;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.subsystems.Subsystems;
import frc.robot.sysid.SysIdDrivetrainLogger;

@AutonomousCommand(name = "Characterize Drivetrain")
public final class CharacterizeDrivetrain extends CommandBase {
  private static final double kDefaultNTUpdateRate = 0.100;
  private static final double kHighFrequencyNTUpdateRate = 0.010;

  private final RomiDrivetrain m_drivetrain;
  private final SysIdDrivetrainLogger m_logger;

  /** Creates a new CharacterizeDrivetrain. */
  public CharacterizeDrivetrain(Subsystems subsystems) {
    m_drivetrain = subsystems.drivetrain;
    m_logger = new SysIdDrivetrainLogger(m_drivetrain);
    addRequirements(m_drivetrain);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_logger.init();
    // NetworkTableInstance.getDefault().setUpdateRate(kHighFrequencyNTUpdateRate);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_logger.logData();

    double leftVoltage = m_logger.getLeftMotorVoltage();
    double rightVoltage = m_logger.getRightMotorVoltage();

    m_drivetrain.setMotorVoltages(leftVoltage, rightVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // NetworkTableInstance.getDefault().setUpdateRate(kDefaultNTUpdateRate);
    m_drivetrain.stopMotor();
    m_logger.sendData();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriverStation.isDisabled();
  }
}
