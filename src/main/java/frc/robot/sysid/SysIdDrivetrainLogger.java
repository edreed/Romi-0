// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sysid;

import frc.robot.subsystems.RomiDrivetrain;

/** Add your docs here. */
public class SysIdDrivetrainLogger extends SysIdLogger {
    private final RomiDrivetrain m_drivetrain;

    private double m_leftMotorVoltage;
    private double m_rightMotorVoltage;

    /**
     * Constructs an instance of a logger to gather data on a drivetrain.
     * 
     * @param wheelModuleRadius The distance in meters from the center of the robot
     *                          frame to the wheel module. This is used to calculate
     *                          the distance driven by each side of the robot to
     *                          simulate expected data for a differential drive.
     */
    public SysIdDrivetrainLogger(RomiDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    @Override
    protected boolean isWrongMechanism() {
        String mechanism = getMechanism();

        return !mechanism.equals("Drivetrain") && !mechanism.equals("Drivetrain (Angular)");
    }

    /**
     * Returns the voltage to apply to the left motor.
     * 
     * @return The left motor voltage.
     */
    public double getLeftMotorVoltage() {
        return m_leftMotorVoltage;
    }

    /**
     * Returns the voltage to apply to the right motor.
     * 
     * @return The right motor voltage.
     */
    public double getRightMotorVoltage() {
        return m_rightMotorVoltage;
    }

    /**
     * Logs data for the SysID tool for a Swerve drivetrain.
     * 
     * @param measuredPosition    The measured position of the robot in the
     *                            direction of movement.
     * @param measuredVelocity    The measured velocity of the robot, in meters per
     *                            second.
     * @param measuredAngle       The measured orientation of the robot, in radians.
     * @param measuredAngularRate The measured angular rate of the change of the
     *                            robot, in radians per second.
     */
    public void logData() {
        updateData();

        boolean rotating = isRotating();
        double timestamp = getTimestamp(); 
        double leftPosition = m_drivetrain.getLeftDistanceMeters();
        double rightPosition = m_drivetrain.getRightDistanceMeters();
        double leftVelocity = m_drivetrain.getLeftVelocity();
        double rightVelocity = m_drivetrain.getRightVelocity();
        double heading = m_drivetrain.getGyroAngle().getRadians();
        double angularRate = m_drivetrain.getGyroAngularRate();

        // The data format for a general drivetrain is described here in
        // https://github.com/wpilibsuite/sysid/blob/main/docs/data-collection.md#drivetrain
        addData(timestamp,
                m_leftMotorVoltage,
                m_rightMotorVoltage,
                leftPosition,
                rightPosition,
                leftVelocity,
                rightVelocity,
                heading,
                angularRate);

        double motorVoltage = getMotorVoltage();

        m_leftMotorVoltage = (rotating ? -1 : 1) * motorVoltage;
        m_rightMotorVoltage = motorVoltage;
    }

    @Override
    public void reset() {
        super.reset();

        m_drivetrain.resetPosition();
        m_leftMotorVoltage = 0.0;
        m_rightMotorVoltage = 0.0;
    }
}
