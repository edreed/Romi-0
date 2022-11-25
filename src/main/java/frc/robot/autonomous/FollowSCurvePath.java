// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.List;

import com.nrg948.autonomous.AutonomousCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowTrajectory;
import frc.robot.subsystems.Subsystems;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
@AutonomousCommand(name = "Follow S-Curve Path")
public final class FollowSCurvePath extends SequentialCommandGroup {
  private static final Pose2d INITIAL_POSE = new Pose2d(0, 0, new Rotation2d(0));
  private static final List<Translation2d> WAYPOINTS = List.of(
      new Translation2d(0.5, 0.5),
      new Translation2d(1.0, -0.5));
  private static final Pose2d FINAL_POSE = new Pose2d(1.5, 0, new Rotation2d(0));

  /** Creates a new FollowSCurvePath. */
  public FollowSCurvePath(Subsystems subsystems) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> subsystems.drivetrain.resetPosition(INITIAL_POSE)),
        FollowTrajectory.fromWaypoints(
            subsystems.drivetrain,
            INITIAL_POSE,
            WAYPOINTS,
            FINAL_POSE));
  }
}
