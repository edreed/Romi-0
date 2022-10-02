// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DigitalOutputPort;

// A subsystem that simulates the Robot Signal Light using the Romi's
// yellow LED indicator.
public class RSL extends SubsystemBase {
  public static final double kRslBlinkPeriod = 0.5;

  private DigitalOutput m_yellowLED = new DigitalOutput(DigitalOutputPort.YellowLED.get());
  private Timer m_timer = new Timer();
  private Boolean wasDisabled = true;

  /** Creates a new RSL. */
  public RSL() {
    m_yellowLED.set(true);
  }

  @Override
  public void periodic() {
    if (wasDisabled) {
      if (DriverStation.isEnabled()) {
        wasDisabled = false;
        m_yellowLED.set(true);
        m_timer.reset();
        m_timer.start();
      }
    } else {
      if (DriverStation.isDisabled()) {
        wasDisabled = true;
        m_yellowLED.set(true);
        m_timer.stop();
      }
    }

    if (m_timer.advanceIfElapsed(kRslBlinkPeriod)) {
      m_yellowLED.set(!m_yellowLED.get());
    }
  }
}
