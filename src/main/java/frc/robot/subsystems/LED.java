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
  private int fade = 0;
  private int direction = 0;
  private AddressableLED m_led = new AddressableLED(0);
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(45);
  private int i = 0;
  public int time = 0;
  private int mode = 0;
  public boolean isEnabled = false;

  public LED() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();
  }

  @Override
  public void periodic() {
    if (isEnabled == false) {
      setLedFade();
    } 
  }

  // private void rainbow() {
  // // For every pixel
  // for (var i = 0; i < m_ledBuffer.getLength(); i++) {
  // // Calculate the hue - hue is easier for rainbows because the color
  // // shape is a circle so only one value needs to precess
  // final var hue = (m_rainbowFirstPixelHue + (i * 180 /
  // m_ledBuffer.getLength())) % 180;
  // // Set the value
  // m_ledBuffer.setHSV(i, hue, 255, 128);
  // }
  // // Increase by to make the rainbow "move"
  // m_rainbowFirstPixelHue += 1;
  // // Check bounds
  // m_rainbowFirstPixelHue %= 180;
  // }

  public void setLedCube() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 255, 0, 255);
    }
    m_led.setData(m_ledBuffer);
  }

  public void setLedCone() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 255, 0);
    }
    m_led.setData(m_ledBuffer);
  }

  public void setLEDBlack() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }
    m_led.setData(m_ledBuffer);
  }

  public void setLEDGreen() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 255);
    }
    m_led.setData(m_ledBuffer);
  }

  public void setLedColor() {
    if (DriverStation.getAlliance() == Alliance.Blue)
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, 255, 0);
      }
    else {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 0, 0);
      }
    }
    m_led.setData(m_ledBuffer);
  }

  public void setLedFade() {
    if(time<10000) {
      time ++;
    if (DriverStation.getAlliance() == Alliance.Blue) {
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
      for (i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setHSV(i, 0, 255, fade);
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
    }
  } else {
      time ++;

      if(direction == 1){
        i++;
        m_ledBuffer.setHSV(i,0,255,255);
        m_ledBuffer.setHSV(i-1,0,255,0);
      } else {
        i--;
        m_ledBuffer.setHSV(i,0,255,255);
        m_ledBuffer.setHSV(i+1,0,255,0);
      }
      if(i > 36){
        direction = 0;
      } else if(i<3){
        direction = 1;
      }

  }
    m_led.setData(m_ledBuffer);
  }

  public void setLEDOrange(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 128, 0);
    }
    m_led.setData(m_ledBuffer);
  }

  public Command fadeCommand() {
    return this.run(() -> setLedFade());
  }

  public Command setOrange() {
    return this.runEnd(() -> setLEDOrange(), () -> setLedColor());
  }

  public Command actuallyCone() {
    return this.runEnd(() -> setLedCone(), () -> setLedColor());
  }

  public Command actuallyCube() {
    return this.runEnd(() -> setLedCube(), () -> setLedColor());
  }

  public Command accyGreem() {
    return this.runEnd(() -> setLEDGreen(), () -> setLedColor());
  }

  public Command actuallyColor() {
    return this.runEnd(() -> setLedColor(), () -> setLedColor());
  }

  public void setMode(int mode) {
    this.mode = mode;
  }

}
