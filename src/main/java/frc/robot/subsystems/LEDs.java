// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.epilogue.Logged;
import org.wpilib.hardware.led.AddressableLED;
import org.wpilib.hardware.led.AddressableLEDBuffer;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.hardware.led.LEDPattern.GradientType;
import org.wpilib.framework.RobotBase;
import org.wpilib.util.Color;
import org.wpilib.command2.SubsystemBase;
import frc.robot.constants.LED;
import frc.robot.constants.LED.LED_STATES;
import frc.robot.simulation.LEDSim;
import java.util.Map;
import java.util.function.DoubleSupplier;

public class LEDs extends SubsystemBase {
  private LED_STATES currentState = LED_STATES.DISABLED;

  // We'll be using a WS2812 LED strip controlled through PWM
  private AddressableLED m_led = new AddressableLED(LED.kPWMPort);

  private AddressableLEDBuffer m_ledBuffer;
  // The buffer we apply patterns to before copying to m_ledBuffer.
  // This allows us to create a buffer larger than the number of LEDs
  private AddressableLEDBuffer workingBuffer;
  private LEDPattern m_currentPattern =
      LEDPattern.rainbow(255, 127).scrollAtRelativeVelocity(Percent.per(Second).of(50));

  private LEDSim m_ledSim2d;

  public LEDs() {
    m_ledBuffer = new AddressableLEDBuffer(LED.kLEDCount);
    workingBuffer = new AddressableLEDBuffer(LED.kLEDCount * 2);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    // m_led.start();

    if (RobotBase.isSimulation()) {
      m_ledSim2d = new LEDSim(m_ledBuffer, LEDSim.Layout.HORIZONTAL);
    }
  }

  public void setState(LED_STATES state, DoubleSupplier shooterProgress) {
    LEDPattern base;
    if (state != currentState) {
      currentState = state;
      switch (currentState) {
        case DISABLED:
          base = LEDPattern.rainbow(255, 127);
          m_currentPattern = base.scrollAtRelativeVelocity(Percent.per(Second).of(50));
          workingBuffer = new AddressableLEDBuffer(LED.kLEDCount);
          break;
        case IDLE:
          base = LEDPattern.gradient(GradientType.CONTINUOUS, Color.GREEN, Color.BLACK);
          m_currentPattern = base.scrollAtRelativeVelocity(Percent.per(Second).of(50));
          workingBuffer = new AddressableLEDBuffer(LED.kLEDCount * 2);
          break;
        case IDLE_CAN_ERROR:
          base = LEDPattern.gradient(GradientType.CONTINUOUS, Color.ORANGE, Color.BLACK);
          m_currentPattern = base.scrollAtRelativeVelocity(Percent.per(Second).of(50));
          workingBuffer = new AddressableLEDBuffer(LED.kLEDCount * 2);
          break;
        case RED_SHIFT_END:
          base = LEDPattern.solid(Color.RED);
          m_currentPattern = base.breathe(Seconds.of(0.5));
          workingBuffer = new AddressableLEDBuffer(LED.kLEDCount);
          break;
        case BLUE_SHIFT_END:
          base = LEDPattern.solid(Color.BLUE);
          m_currentPattern = base.breathe(Seconds.of(0.5));
          workingBuffer = new AddressableLEDBuffer(LED.kLEDCount);
          break;
        case INTAKING:
          base =
              LEDPattern.steps(
                  Map.of(
                      0,
                      Color.BLACK,
                      0.25,
                      Color.YELLOW,
                      0.5,
                      Color.BLACK,
                      0.75,
                      Color.YELLOW)); // Yeah, you know what it is
          m_currentPattern = base.scrollAtRelativeVelocity(Percent.per(Second).of(100));
          workingBuffer = new AddressableLEDBuffer(LED.kLEDCount);
          break;
        case SHOOTING:
          base = LEDPattern.steps(Map.of(0, Color.YELLOW, 0.33, Color.RED, 0.67, Color.BLUE));
          LEDPattern mask = LEDPattern.progressMaskLayer(() -> 1 - shooterProgress.getAsDouble());
          m_currentPattern = base.mask(mask);
          workingBuffer = new AddressableLEDBuffer(LED.kLEDCount);
          break;
      }
    }
  }

  public void setState(LED_STATES state) {
    setState(state, () -> 0.0);
  }

  public String getDataString() {
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      Color c = m_ledBuffer.getLED(i);
      int r = (int) Math.round(Math.max(0, Math.min(1, c.red)) * 255);
      int g = (int) Math.round(Math.max(0, Math.min(1, c.green)) * 255);
      int b = (int) Math.round(Math.max(0, Math.min(1, c.blue)) * 255);
      sb.append(String.format("LED %d: R=%d G=%d B=%d\n", i, r, g, b));
    }
    return sb.toString();
  }

  @Logged(name = "LED State", importance = Logged.Importance.DEBUG)
  public LED_STATES getState() {
    return currentState;
  }

  @Override
  public void periodic() {
    m_currentPattern.applyTo(workingBuffer);
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, workingBuffer.getLED(i));
    }
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void simulationPeriodic() {
    m_ledSim2d.update();
  }
}
