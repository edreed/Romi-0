// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DigitalInputPort;
import frc.robot.Constants.PWMPort;

public class RomiDrivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterMeters = 0.070; // 70 mm

  // Set up the left and right motors.
  private final Spark m_leftMotor = new Spark(PWMPort.LeftMotor.get());
  private final Spark m_rightMotor = new Spark(PWMPort.RightMotor.get());

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Set up the encoders and gyro.
  private final Encoder m_leftEncoder = new Encoder(
      DigitalInputPort.LeftEncoderA.get(),
      DigitalInputPort.LeftEncoderB.get());
  private final Encoder m_rightEncoder = new Encoder(
      DigitalInputPort.RightEncoderA.get(),
      DigitalInputPort.RightEncoderB.get());
  private final Gyro m_gyro = new RomiGyro();

  // Set up the differential drive odometry
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use meters as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    resetPosition();

    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void stopMotor() {
    m_diffDrive.stopMotor();
  }

  /** Resets the initial position to the origin with a heading of 0°. */
  public void resetPosition() {
    resetPosition(new Pose2d());
  }

  /**
   * Resets the initial position to the specified pose.
   * 
   * @param initialPose The initial position of the robot.
   */
  public void resetPosition(Pose2d initialPose) {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_gyro.reset();

    Rotation2d gyroOffset = m_gyro.getRotation2d();

    m_odometry.resetPosition(initialPose, gyroOffset);
  }

  /**
   * Returns the heading of the robot.
   * 
   * @return The heading of the robot in degress using the NWU axis convention.
   *         An increasing value indicates rotation in the counter-clockwise
   *         direction.
   */
  public Rotation2d getHeading() {
    return m_odometry.getPoseMeters().getRotation();
  }

  /**
   * Returns the distance travelled by the left wheel in meters.
   * 
   * @return The distance travelled by the left wheel in meters.
   */
  public double getLeftDistanceMeters() {
    return m_leftEncoder.getDistance();
  }

  /**
   * Returns the distance travelled by the right wheel in meters.
   * 
   * @return The distance travelled by the right wheel in meters.
   */
  public double getRightDistanceMeters() {
    return m_rightEncoder.getDistance();
  }

  /**
   * Returns the current pose of the robot.
   * 
   * @return The current pose of the robot.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    Rotation2d heading = m_gyro.getRotation2d();
    double leftDistance = m_leftEncoder.getDistance();
    double rightDistance = m_rightEncoder.getDistance();

    m_odometry.update(heading, leftDistance, rightDistance);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
