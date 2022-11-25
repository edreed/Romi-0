// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.RomiDrivetrain;

/** Add your docs here. */
public class FollowTrajectory extends RamseteCommand {

  private final RomiDrivetrain m_drivetrain;

  public static FollowTrajectory fromWaypoints(
      RomiDrivetrain drivetrain,
      Pose2d start,
      List<Translation2d> waypoints,
      Pose2d end,
      TrajectoryConstraint... constraints) {
    TrajectoryConfig config = drivetrain.getTrajectoryConfig()
        .addConstraints(List.of(constraints));
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);

    return new FollowTrajectory(drivetrain, trajectory);
  }

  public static FollowTrajectory fromPathweaverJson(RomiDrivetrain drivetrain, Path path) throws IOException {
    Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(path);

    return new FollowTrajectory(drivetrain, trajectory);
  }

  public FollowTrajectory(RomiDrivetrain drivetrain, Trajectory trajectory) {
    super(
        trajectory,
        drivetrain::getPose,
        new RamseteController(),
        drivetrain.getKinematics(),
        drivetrain::setWheelSpeeds,
        drivetrain);
    m_drivetrain = drivetrain;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_drivetrain.stopMotor();
  }
}
