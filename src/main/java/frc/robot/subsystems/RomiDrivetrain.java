// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.nrg948.preferences.RobotPreferences.DoubleValue;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DigitalInputPort;
import frc.robot.Constants.PWMPort;
import frc.robot.RomiStatus;

@RobotPreferencesLayout(groupName = "Drivetrain", column = 2, row = 0, width = 2, height = 3)
public class RomiDrivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterMeters = 0.070; // 70 mm
  private static final double kWheelTrackWidthMeters = 0.141; // 141 mm

  // The theoretical max speed based on the published specs of no-load RPM
  // is (150 RPM * 0.070m * π) / 60s = 0.5497787143782139. Set the max speed
  // in code to something below that to ensure we can hit it.
  public static final double kMaxSpeed = 0.5; // m/s

  // The max angular speed can be determined by dividing max speed by the
  // half the track width. (The track width can be thought of as the diameter
  // of the circle inscribed by the two wheels when rotating in the opposite
  // direction.)
  public static final double kMaxAngularSpeed = (2 * kMaxSpeed) / kWheelTrackWidthMeters;

  // Feed forward constants.
  @RobotPreferencesValue
  public static final DoubleValue kFeedForwardS = new DoubleValue("Drivetrain", "Feed Forward S", 1.0); // A guess
  @RobotPreferencesValue
  public static final DoubleValue kFeedForwardV = new DoubleValue(
      "Drivetrain", "Feed Forward V", RomiStatus.getMaxBatteryVoltage() / kMaxSpeed); // Theoretical value

  // Wheel speed PID constants.
  @RobotPreferencesValue
  public static final DoubleValue kWheelSpeedP = new DoubleValue("Drivetrain", "Wheel Speed P", 1.0);
  @RobotPreferencesValue
  public static final DoubleValue kWheelSpeedI = new DoubleValue("Drivetrain", "Wheel Speed I", 0.0);
  @RobotPreferencesValue
  public static final DoubleValue kWheelSpeedD = new DoubleValue("Drivetrain", "Wheel Speed D", 0.0);

  // Set up the left and right motors.
  private final Spark m_leftMotor = new Spark(PWMPort.LeftMotor.get());
  private final Spark m_rightMotor = new Spark(PWMPort.RightMotor.get());

  // Set up the encoders and gyro.
  private final Encoder m_leftEncoder = new Encoder(
      DigitalInputPort.LeftEncoderA.get(),
      DigitalInputPort.LeftEncoderB.get());
  private final Encoder m_rightEncoder = new Encoder(
      DigitalInputPort.RightEncoderA.get(),
      DigitalInputPort.RightEncoderB.get());
  private final Gyro m_gyro = new RomiGyro();

  // Set up the differential drive odometry.
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

  // Set up kinematics, PID controllers and feedforward.
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kWheelTrackWidthMeters);
  private final PIDController m_leftSpeedPidController = new PIDController(
      kWheelSpeedP.getValue(), kWheelSpeedI.getValue(), kWheelSpeedD.getValue());
  private final PIDController m_rightSpeedPidController = new PIDController(
      kWheelSpeedP.getValue(), kWheelSpeedI.getValue(), kWheelSpeedD.getValue());
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(
      kFeedForwardS.getValue(), kFeedForwardV.getValue());

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use meters as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    resetPosition();

    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);
  }

  /**
   * Drives the robot using arcade style of control.
   * 
   * @param xAxis The robot's speed along the x-axis relative to the robot frame.
   *              The value is in the range [-1.0..1.0] representing a fraction of
   *              the robot's maximum speed. A positive value results forward
   *              motion.
   * @param zAxis The robot's rotational speed around the z-axis. The value is in
   *              the range [-1.0..1.0] representing a fraction of the robot's
   *              maximum rotational speed. A positive value results in
   *              counter-clockwise motion.
   */
  public void arcadeDrive(double xAxis, double zAxis) {
    final double xSpeed = MathUtil.clamp(xAxis, -1.0, 1.0) * kMaxSpeed;
    final double zRotation = MathUtil.clamp(zAxis, -1.0, 1.0) * kMaxAngularSpeed;

    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, zRotation));

    setWheelSpeeds(wheelSpeeds);
  }

  /**
   * Sets the desired wheel speeds.
   * 
   * @param wheelSpeeds The desired wheel speeds.
   */
  public void setWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
    final double leftFeedforward = m_feedforward.calculate(wheelSpeeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(wheelSpeeds.rightMetersPerSecond);

    final double leftOutput = m_leftSpeedPidController.calculate(
        m_leftEncoder.getRate(), wheelSpeeds.leftMetersPerSecond);
    final double rightOutput = m_rightSpeedPidController.calculate(
        m_rightEncoder.getRate(), wheelSpeeds.rightMetersPerSecond);

    setMotorVoltages(leftOutput + leftFeedforward, rightOutput + rightFeedforward);
  }

  /**
   * Sets the voltage output of the left and right motor controllers. This method
   * compensates for the current bus voltage to ensure that the desired voltage is
   * output even if the battery voltage is below its maximum - highly useful when
   * the voltage outputs are "meaningful" (e.g. they come from a feedforward
   * calculation).
   *
   * <p>
   * NOTE: This method *must* be called regularly in order for voltage
   * compensation to work properly - unlike the ordinary set function, it is not
   * "set it and forget it."
   * 
   * @param leftVoltage  The left voltage output.
   * @param rightVoltage The right voltage output.
   */
  public void setMotorVoltages(double leftVoltage, double rightVoltage) {
    double batteryVoltage = RomiStatus.getBatteryVoltage();

    m_leftMotor.set(MathUtil.clamp(leftVoltage / batteryVoltage, -1.0, 1.0));
    m_rightMotor.set(MathUtil.clamp(rightVoltage / batteryVoltage, -1.0, 1.0));
  }

  /** Stops the motors. */
  public void stopMotor() {
    m_leftMotor.set(0);
    m_rightMotor.set(0);

    m_leftMotor.stopMotor();
    m_rightMotor.stopMotor();

    m_leftSpeedPidController.reset();
    m_rightSpeedPidController.reset();
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
   * Returns the heading of the robot rounded between [-180..180] degrees.
   * 
   * @return The heading of the robot using the NWU axis convention. An increasing
   *         value indicates rotation in the counter-clockwise direction.
   */
  public Rotation2d getHeading() {
    return m_odometry.getPoseMeters().getRotation();
  }

  /**
   * Returns the continuous heading of the robot as reported by the gyro.
   * 
   * @return The heading of the robot using the NWU axis convention. An increasing
   *         value indicates rotation in the counter-clockwise direction.
   */
  public Rotation2d getGyroAngle() {
    return m_gyro.getRotation2d();
  }

  /**
   * Returns the rate of rotation of the gyro.
   * 
   * @return The rate of rotation of the gyro.
   */
  public double getGyroAngularRate() {
    return -m_gyro.getRate();
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
   * Returns the velocity of the left wheel.
   * 
   * @return The velocity of the left wheel in meters per second.
   */
  public double getLeftVelocity() {
    return m_leftEncoder.getRate();
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
   * Returns the velocity of the right wheel.
   * 
   * @return The velocity of the right wheel in meters per second.
   */
  public double getRightVelocity() {
    return m_rightEncoder.getRate();
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
