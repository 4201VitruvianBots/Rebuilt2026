// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import org.wpilib.hardware.led.AddressableLEDBuffer;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.util.Color8Bit;

/** A class for simulating LEDs using a Mechanism2d object. */
public class LEDSim {
  public enum Layout {
    HORIZONTAL,
    VERTICAL
  }

  public enum Direction {
    LEFT_TO_RIGHT,
    RIGHT_TO_LEFT
  }

  private AddressableLEDBuffer m_ledBuffer;

  private Mechanism2d m_mech2d;

  private MechanismLigament2d[] m_ledLigaments;

  public LEDSim(AddressableLEDBuffer ledBuffer, Layout layout) {
    m_ledBuffer = ledBuffer;
    if (layout == Layout.HORIZONTAL) {
      m_mech2d = new Mechanism2d(ledBuffer.getLength() * 10, 10);
    } else {
      m_mech2d = new Mechanism2d(10, ledBuffer.getLength() * 10);
    }

    m_ledLigaments = new MechanismLigament2d[ledBuffer.getLength()];
    if (layout == Layout.HORIZONTAL) {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        m_ledLigaments[i] =
            m_mech2d
                .getRoot("LED " + i, i * 10, 5)
                .append(new MechanismLigament2d("LED " + i, 10, 0));
      }
    } else {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        m_ledLigaments[i] =
            m_mech2d
                .getRoot("LED " + i, 5, i * 10)
                .append(new MechanismLigament2d("LED " + i, 10, 0));
      }
    }
    SmartDashboard.putData("LEDSim", m_mech2d);
  }

  public void update() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledLigaments[i].setColor(new Color8Bit(m_ledBuffer.getLED(i)));
    }
  }
}
