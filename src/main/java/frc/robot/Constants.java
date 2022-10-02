// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static enum DigitalInputPort {
        ButtonA(0),
        ButtonB(1),
        ButtonC(2),
        LeftEncoderA(4),
        LeftEncoderB(5),
        RightEncoderA(6),
        RightEncoderB(7);

        private int m_port;

        DigitalInputPort(int port) {
            m_port = port;
        }

        public int get() {
            return m_port;
        }
    }

    public static enum DigitalOutputPort {
        GreenLED(1),
        RedLED(2),
        YellowLED(3);

        private int m_port;

        DigitalOutputPort(int port) {
            m_port = port;
        }

        public int getPort() {
            return m_port;
        }
    }

    public static enum PWMPort {
        LeftMotor(0),
        RightMotor(1);

        private int m_port;

        PWMPort(int port) {
            m_port = port;
        }

        public int get() {
            return m_port;
        }
    }
}
