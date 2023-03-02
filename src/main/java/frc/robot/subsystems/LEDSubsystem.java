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
  }

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private int NUMBER_LED = 85;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue;

  private LEDMode ledMode;

  public LEDSubsystem() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);

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

  // BIG buggy
  // Make an if-statement or some periodic thing so that when a button is pressed the for loop
  // breaks
  // Then the light should change to the desired color (either yellow or purple)
  // This is so that the actual rainbow stops changing the incoming changes

  public void setLedMode(LEDMode ledMode) {
    this.ledMode = ledMode;
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

  // Green and blue are swapped
  // It is Red, Blue, Green (RBG)
  // When changing colors make sure to correct

  public void yellow() {
    generalLED(0, NUMBER_LED, 255, 70, 0);
  }

  public void purple() {
    generalLED(0, NUMBER_LED, 255, 0, 255);
  }

  public void clear() {
    generalLED(0, NUMBER_LED, 0, 0, 0);
  }

  @Override
  public void periodic() {
    System.out.println(ledMode);
    switch (ledMode) {
      case RAINBOW:
        rainbow();
        break;
      case EMERGENCY:
        purple();
        break;
    }

    m_led.setData(m_ledBuffer);
  }

  public void generalLED(int startLED, int endLED, int redColor, int greenColor, int blueColor) {
    for (var i = startLED; i < endLED; i++) {
      m_ledBuffer.setRGB(i, redColor, blueColor, greenColor);
    }
  }
}

// public class LEDSubsystem extends SubsystemBase {
//   private AddressableLED m_led;
//   private AddressableLEDBuffer m_ledBuffer;
//   private int m_rainbowFirstPixelHue;
//   private int iteration = 0;
//   private boolean entering = true;
//   private long lastTime = System.currentTimeMillis();
//   private int color = 0;
//   private int mode = 0;
//   private int percentage = 0;
//   private int start = 0;

//   public static final int MODE_WAVE = 0;
//   public static final int MODE_BAR = 1;
//   public static final int MODE_RAINBOW = 2;
//   public static final int MODE_SOLID = 3;
//   public static final int MODE_OFF = 4;

//   public LEDSubsystem(int port, int leds) {
//     // initialize all relevant objects
//     m_led = new AddressableLED(port);

//     m_ledBuffer = new AddressableLEDBuffer(leds);
//     m_led.setLength(m_ledBuffer.getLength());

//     m_led.setData(m_ledBuffer);
//     m_led.start();
//   }

//   public LEDSubsystem(int port, int leds, int start) {
//     // initialize all relevant objects
//     m_led = new AddressableLED(port);
//     this.start = start;

//     m_ledBuffer = new AddressableLEDBuffer(leds + this.start);
//     m_led.setLength(m_ledBuffer.getLength());

//     m_led.setData(m_ledBuffer);
//     m_led.start();
//   }

//   public void update() {
//     // System.out.println("mode - " + this.mode);
//     if (this.mode == 0 || this.mode == 3) {
//       this.wave();
//     } else if (this.mode == 1) {
//       this.bar();
//     } else if (this.mode == 2) {
//       this.rainbow();
//       m_led.setData(m_ledBuffer);
//     } else if (this.mode == 4) {
//       for (int i = 0; i < this.m_ledBuffer.getLength() - start; i++) {
//         m_ledBuffer.setHSV(i + start, 0, 255, 0);
//       }
//       m_led.setData(m_ledBuffer);
//     }
//   }

//   private void rainbow() {
//     // For every pixel
//     for (var i = 0; i < m_ledBuffer.getLength() - start; i++) {
//       // Calculate the hue - hue is easier for rainbows because the color
//       // shape is a circle so only one value needs to precess
//       final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
//       // Set the value
//       m_ledBuffer.setHSV(i + start, hue, 255, 128);
//     }
//     // Increase by to make the rainbow "move"
//     m_rainbowFirstPixelHue += 3;
//     // Check bounds
//     m_rainbowFirstPixelHue %= 180;
//   }

//   private void wave() {
//     // set the hue to the selected color

//     int hue = 0;
//     switch (color) {
//       case Colors.RED:
//         hue = 0;
//         break;

//       case Colors.PINK:
//         hue = 175;
//         break;

//       case Colors.PURPLE:
//         hue = 150;
//         break;

//       case Colors.BLUE:
//         hue = 120;
//         break;

//       case Colors.CYAN:
//         hue = 100;
//         break;

//       case Colors.GREEN:
//         hue = 45;
//         break;

//       case Colors.YELLOW:
//         hue = 20;
//         break;

//       case Colors.ORANGE:
//         hue = 11;
//         break;
//     }
//     // System.out.println("LED H" + hue + " - " + color );
//     if (this.mode == 0) {
//       // draw leds as animation
//       if (this.entering) {
//         for (int i = 0; i < this.iteration; i++) {
//           m_ledBuffer.setHSV(i + start, hue, 255, 255);
//         }

//         for (int i = this.iteration; i < this.m_ledBuffer.getLength() - iteration - start; i++) {
//           m_ledBuffer.setHSV(i + start, hue, 255, 0);
//         }

//         for (int i = this.m_ledBuffer.getLength() - iteration - start;
//             i < this.m_ledBuffer.getLength() - start;
//             i++) {
//           m_ledBuffer.setHSV(i + start, hue, 255, 255);
//         }
//       } else {
//         for (int i = 0; i < this.iteration; i++) {
//           m_ledBuffer.setHSV(i + start, hue, 255, 0);
//         }

//         for (int i = this.iteration; i < this.m_ledBuffer.getLength() - iteration - start; i++) {
//           m_ledBuffer.setHSV(i + start, hue, 255, 255);
//         }

//         for (int i = this.m_ledBuffer.getLength() - iteration - start;
//             i < this.m_ledBuffer.getLength() - start;
//             i++) {
//           m_ledBuffer.setHSV(i + start, hue, 255, 0);
//         }
//       }

//       if (System.currentTimeMillis() - 25 > this.lastTime) {
//         this.iteration++;
//         this.lastTime = System.currentTimeMillis();
//       }
//       if (this.iteration == (this.m_ledBuffer.getLength() / 2) + 1) {
//         this.iteration = 0;
//         this.entering = !this.entering;
//       }
//     } else {
//       for (int i = 0; i < this.m_ledBuffer.getLength() - start; i++) {
//         this.m_ledBuffer.setHSV(i + start, hue, 255, 255);
//       }
//     }

//     m_led.setData(m_ledBuffer);
//   }

//   private void bar() {
//     // set the hue to the selected color
//     int hue = 0;
//     if (this.percentage > 33) {
//       hue = 11;
//     }

//     if (this.percentage > 66) {
//       hue = 20;
//     }

//     if (this.percentage >= 99) {
//       hue = 45;
//     }

//     int half = this.m_ledBuffer.getLength() / 2;
//     double percentage = this.percentage / 100.0;
//     if (percentage > 1.0) {
//       percentage = 1.0;
//     }

//     double size = percentage * half;
//     long round = Math.round(size);
//     for (int i = 0; i < this.m_ledBuffer.getLength() - start; i++) {
//       m_ledBuffer.setHSV(i + start, 0, 0, 0);
//     }

//     for (long i = half - round; i < half + round; i++) {
//       m_ledBuffer.setHSV((int) i + start, hue, 255, 255);
//     }

//     m_led.setData(m_ledBuffer);
//   }

//   public void setColor(int newcolor) {
//     this.color = newcolor;
//   }

//   public int getColor() {
//     return this.color;
//   }

//   public void setPercentage(int percentage) {
//     this.percentage = percentage;
//   }

//   public void setMode(int newmode) {
//     this.mode = newmode;
//   }

//   public int getMode() {
//     return this.mode;
//   }

//   public class LEDS {
//     public static final int PORT = 0;
//     public static final int COUNT = 5;
//     public static final int FLASH_DELAY = 5;

//     public class Colors {
//       public static final int RED = 0;
//       public static final int PINK = 1;
//       public static final int PURPLE = 2;
//       public static final int BLUE = 3;
//       public static final int CYAN = 4;
//       public static final int GREEN = 5;
//       public static final int YELLOW = 6;
//       public static final int ORANGE = 7;
//     }
//   }
// }
