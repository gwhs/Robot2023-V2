// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  public enum LEDMode {
    RAINBOW,
    EMERGENCY,
    YELLOW,
    PURPLE,
  }

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private int NUMBER_LED = 85;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue;
  private double m_brightness = 1;

  // Change for different PWM ports
  private int ledPortNumber = 9;

  private LEDMode ledMode;

  public LEDSubsystem() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(ledPortNumber);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(85);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    ledMode = LEDMode.RAINBOW;
  }

  public void setLedMode(LEDMode ledMode) {
    this.ledMode = ledMode;
  }

  public LEDMode getLedMode() {
    return ledMode;
  }

  public void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }

    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 4;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  public void yellow() {
    m_brightness += .05;

    if (m_brightness > 1) {
      m_brightness = 0.01;
    }
    generalLED(0, NUMBER_LED, 255, 70, 0);
  }

  public void purple() {
    m_brightness += .05;

    if (m_brightness > 1) {
      m_brightness = 0.01;
    }

    generalLED(0, NUMBER_LED, 255, 0, 255);
  }

  public void emergency() {
    generalLED(0, NUMBER_LED, 255, 0, 0);
  }

  public void clear() {
    generalLED(0, NUMBER_LED, 0, 0, 0);
  }

  @Override
  public void periodic() {
    // System.out.println(ledMode);
    switch (ledMode) {
      case RAINBOW:
        rainbow();
        break;
      case EMERGENCY:
        emergency();
        break;
      case YELLOW:
        yellow();
        break;
      case PURPLE:
        purple();
        break;
    }

    m_led.setData(m_ledBuffer);
  }

  public void generalLED(int startLED, int endLED, int redColor, int greenColor, int blueColor) {
    for (var i = startLED; i < endLED; i++) {
      m_ledBuffer.setRGB(
          i,
          (int) (redColor * m_brightness),
          (int) (blueColor * m_brightness),
          (int) (greenColor * m_brightness));
    }
  }
}
