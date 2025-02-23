// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Colors;
import frc.robot.LimelightHelpers;

public class LEDSubsystem extends SubsystemBase {
  //Creates a new subsystem for the LEDs

  Spark blinkin;

  boolean discoOn = false;

  private AddressableLED m_led;

  public LEDSubsystem() {

    blinkin = new Spark(0);
  }
  
  //These Adressable LED buffers are for seperateing individual strings of LEDs into multiple sections
  //This should be the 5 meter (12 Volt) strip of LEDs
  AddressableLEDBuffer m_buffer_12Volt = new AddressableLEDBuffer(300);

  // This is the view for the section of the strip on the left side of the robot.
  // This section spans LEDs from index 0 through index 249, inclusive.
  AddressableLEDBufferView m_1st_12V_section = m_buffer_12Volt.createView(0, 149);

  // This is the section of the strip on the right side of the robot.
  // This section spans LEDs from index 250 through index 499, inclusive.
  // The "reversed()" 2 lines down was on the example but probably isn't useful
  AddressableLEDBufferView m_2nd_12V_section = m_buffer_12Volt.createView(150, 299);
  //.reversed()

  //This should control the 1 meter (5 Volt) strip of LEDs
  AddressableLEDBuffer m_buffer_5Volt = new AddressableLEDBuffer(60);

  //This is the 1st section of the 1 meter strip (currently out of 3)
  AddressableLEDBufferView m_1st_5V_section = m_buffer_5Volt.createView(0,19);

  //this is the 2nd out of 3
  AddressableLEDBufferView m_2nd_5V_section = m_buffer_5Volt.createView(20,39);

  //This is the 3rd out of 3
  AddressableLEDBufferView m_3rd_5V_section = m_buffer_5Volt.createView(40,59);

  public void setLedColor(double color) {
    
    blinkin.set(color);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (discoOn) {

      setRandomColor();
    }

  }

  public void setRandomColor() {
    int min = 1;
    int max = 11;
    int randomNum = (int)(Math.random() * (max - min + 1) + min);
    // randomNum will be between 1 and 11, inclusive

    //This will chose a random color out of the 11 bellow, yes there is more blue than any other color, yes that is intentional
    Double randomColor = null;
    switch (randomNum) {
      case 1: randomColor =  Colors.Hot_Pink;
        break;
      case 2: randomColor =  Colors.Red;
        break;
      case 3: randomColor =  Colors.Orange;
        break;
      case 4: randomColor =  Colors.Yellow;
        break;
      case 5: randomColor =  Colors.Green;
        break;
      case 6: randomColor =  Colors.Aqua;
        break;
      case 7: randomColor =  Colors.Dark_Blue;
        break;
      case 8: randomColor =  Colors.Blue;
        break;
      case 9: randomColor =  Colors.Violet;
        break;
      case 10: randomColor =  Colors.White;
        break;
      case 11: randomColor =  Colors.Black;
        break;
    
    }

    setLedColor(randomColor);
  }

  public void setInitialColor() {

    setLedColor(Constants.Colors.Blue);
  }

  public void ChangeLedColor (int num) {
    discoOn = false;
    switch (num) {
      case 1:  // lined_up
        blinkin.set(Colors.Green);
        break;
      case 2: // LimelightHelpers.getTV("limelight-two");
        blinkin.set(Colors.Red);
        break;
      case 3: // coralSensor (Lidar I think)
        blinkin.set(Colors.Lawn_Green);
        break;
      case 4: // algaeSensor
        blinkin.set(Colors.Blue_Violet);
        break;
      case 5: 
        discoOn = true;
      default:
        setInitialColor();
        break;
    }

  }

}