// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.autonomous.TestAutoCommands;
import frc.robot.subsystems.ArmSubsystems.BoreEncoder;
import frc.robot.subsystems.ArmSubsystems.MagicMotion;

public class LEDSubsystem extends SubsystemBase {

  public enum LEDMode {
    RAINBOW,
    EMERGENCY,
    YELLOW,
    PURPLE,
    TEAMCOLOR,
    // auto
    GREEN,
    ORANGE,
    RED,
  }

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private int NUMBER_LED = 24;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue;
  private double m_brightness = 1;

  // Change for different PWM ports
  private int ledPortNumber = 9;

  private LEDMode ledMode;

  // moving pixel
  private int m_pixelCounter;
  private int m_loopCounter;

  // auto
  private DrivetrainSubsystem drivetrainSubsystem;
  private PoseEstimatorSubsystem poseEstimator;
  private BoreEncoder shaftEncoder;
  private MagicMotion mainArm;
  private int status;

  TestAutoCommands m_status =
      new TestAutoCommands(
          drivetrainSubsystem, poseEstimator, mainArm, shaftEncoder, "HajelPath", status);

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

    ledMode = LEDMode.TEAMCOLOR;
  }

  public void setLedMode(LEDMode ledMode) {
    this.ledMode = ledMode;

    if (status == 1) {
      ledMode = LEDMode.GREEN;
    } else if (status == 2) {
      ledMode = LEDMode.ORANGE;
    } else {
      ledMode = LEDMode.RED;
    }
  }

  public LEDMode getLedMode() {
    return ledMode;
  }

  public int transformColor(int colorOne, int colorTwo, double multiplier) {
    // hwne multiplier is 0, get color one, when color one is on color two
    int value = (int) ((colorOne * multiplier) + (colorTwo * (1 - multiplier)));

    if (value > 255) {
      return 255;
    } else if (value < 0) {
      return 0;
    } else {
      return value;
    }
  }
  // auto
  // public void setStatus() {
  //   if (drivetrainSubsystem
  // }

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

  public void teamColor() {
    // For every pixel
    final int r1 = 255;
    final int g1 = 0;
    final int b1 = 0;

    final int r2 = 0;
    final int g2 = 0;
    final int b2 = 255;

    boolean finish = false;

    int r3;
    int g3;
    int b3;

    // front
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      double t = (double) (i / (m_ledBuffer.getLength() - 1.0));

      r3 = transformColor(r1, r2, t);
      g3 = transformColor(g1, g2, t);
      b3 = transformColor(b1, b2, t);

      // generalLED(i, i + 1, r3, g3, b3);
      m_ledBuffer.setRGB(i, (int) (r3), (int) (b3), (int) (g3));

      if (i == (m_ledBuffer.getLength() - 1)) {
        System.out.println("loop 1");
        r3 = 0;
        g3 = 0;
        b3 = 0;
        // generalLED(0, NUMBER_LED, r1, g2, b3);
        finish = true;
      }
    }

    // reverse
    if (finish) {
      for (var i = NUMBER_LED; i > 0; i--) {
        double t = (double) (i / (m_ledBuffer.getLength() - 1.0));

        r3 = transformColor(r2, r1, t);
        g3 = transformColor(g2, g1, t);
        b3 = transformColor(b2, b1, t);

        // generalLED(i, i + 1, r3, g3, b3);
        m_ledBuffer.setRGB(i, (int) (r3), (int) (b3), (int) (g3));

        if (i == 1) {
          System.out.println("loop 2");
          r3 = 0;
          g3 = 0;
          b3 = 0;
          // generalLED(0, NUMBER_LED, r1, g2, b3);
          finish = false;
        }
      }
    }

    // experiment
  }

  public void movingPixel() {
    m_ledBuffer.setRGB(m_pixelCounter, 0, 255, 0);
    if (m_loopCounter % 5 == 0) {
      m_pixelCounter++;
    }

    if (m_pixelCounter >= NUMBER_LED) {
      m_pixelCounter = 0;
    }
  }

  // auto
  public void green() {
    generalLED(0, NUMBER_LED, 0, 255, 0);
  }

  public void orange() {
    generalLED(0, NUMBER_LED, 255, 50, 0);
  }

  public void red() {
    generalLED(0, NUMBER_LED, 255, 0, 0);
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
      case TEAMCOLOR:
        teamColor();
        break;

        // auto
      case GREEN:
        green();
        break;
      case ORANGE:
        orange();
        break;
      case RED:
        red();
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
  // Hi! I caught your attention! (●'◡'●)❤💚🧡💛💙💜🤎🖤🤍❣💕💞💓💗💖💘💝💟🗿🗿🗿😘💋👄
}
