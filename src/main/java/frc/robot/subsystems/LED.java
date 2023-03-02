// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private int state = 0;
  private int m_rainbowFirstPixelHue = 0;
  private int rainbowtime = 0;
  private int fade = 0;
  private int direction = 0;
  private AddressableLED m_led = new AddressableLED(0);
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(45);
  private int i = 0;
  private int mode = 0;

  public LED() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (mode == 0) {
      switch (state) {
        case 1:
          if (rainbowtime < 300) {
            rainbow();
            rainbowtime++;
          } else {
            rainbowtime = 0;
            i = 0;
            state = 2;
          }
          break;
        case 2:
          if (i < 45) {
            m_ledBuffer.setHSV(i, 0, 0, 0);
            i++;

          } else {
            state = 3;
            i = 0;
          }
          break;
        case 3:
          if (rainbowtime < 300) {
            rainbowtime++;
            for (i = 0; i < m_ledBuffer.getLength(); i++) {
              m_ledBuffer.setHSV(i, 80, 255, fade);
            }
            if (direction == 0) {
              fade += 4;
              if (fade == 128) {
                direction = 1;
              }
            } else if (direction == 1) {
              fade -= 4;
              if (fade == 0) {
                direction = 0;
              }
            }
          } else {
            state = 4;
            i = 0;
            fade = 0;
            direction = 0;
            rainbowtime = 0;
          }
          break;
        case 4:
          if (i < 45) {
            m_ledBuffer.setHSV(i, 0, 0, 0);
            i++;

          } else {
            state = 5;
            i = 0;
          }
          break;
        case 5:
          if ((i < (m_ledBuffer.getLength() - 1))) {
            i++;
            m_ledBuffer.setHSV(i - 1, 0, 0, 0);
            m_ledBuffer.setHSV(i, 80, 255, 128);
          } else {
            state = 6;
          }
          break;
        case 6:
          if ((i > 1)) {
            i--;
            m_ledBuffer.setHSV(i + 1, 0, 0, 0);
            m_ledBuffer.setHSV(i, 80, 255, 128);
          } else {
            state = 7;
            i = 0;
          }
          break;
        case 7:
          if ((i < (m_ledBuffer.getLength() - 1))) {
            i++;
            m_ledBuffer.setHSV(i - 1, 0, 0, 0);
            m_ledBuffer.setHSV(i, 80, 255, 128);
          } else {
            state = 8;
          }
          break;
        case 8:
          if ((i > 1)) {
            i--;
            m_ledBuffer.setHSV(i + 1, 0, 0, 0);
            m_ledBuffer.setHSV(i, 80, 255, 128);
          } else {
            state = 1;
            i = 0;
          }
          break;
        default:
          state = 1;

          break;
      }
    }

      // Set the data
      m_led.setData(m_ledBuffer);
  }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 1;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  public void setLedCube() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 255, 0, 255);
    }
  }

  public void setLedCone() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 255, 0);
    }
  }

  public void setLedBlue() {
    if(DriverStation.getAlliance() == Alliance.Blue)
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, 255, 0);
      }
      else{
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, 255, 0, 0);
        }
      }
  }

  public Command actuallyCone() {
    return this.startEnd(() -> setLedCone(), () -> setLedBlue());
  }

  public Command actuallyCube() {
    return this.startEnd(() -> setLedCube(), () -> setLedBlue());
  }

  public void setMode(int mode) {
    this.mode = mode;
  }

}
