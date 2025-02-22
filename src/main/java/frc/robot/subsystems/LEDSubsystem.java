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
import frc.robot.LimelightHelpers;

public class LEDSubsystem extends SubsystemBase {
  //Creates a new subsystem for the LEDs

  Spark blinkin;

  private AddressableLED m_led;

  public LEDSubsystem() {

    blinkin = new Spark(0);
  }
  
  //These Adressable LED buffers are for seperateing individual strings of LEDs into multiple sections
  //This should be the 5 meter (12 Volt) strip of LEDs
  AddressableLEDBuffer m_buffer_12Volt = new AddressableLEDBuffer(500);

  // This is the view for the section of the strip on the left side of the robot.
  // This section spans LEDs from index 0 through index 249, inclusive.
  AddressableLEDBufferView m_1st_12V_section = m_buffer_12Volt.createView(0, 249);

  // This is the section of the strip on the right side of the robot.
  // This section spans LEDs from index 250 through index 499, inclusive.
  // The "reversed()" three lines down was on the example but probably isn't useful
  AddressableLEDBufferView m_2nd_12V_section = m_buffer_12Volt.createView(250, 499);
  /*.reversed()*/

  //This should control the 1 meter (5 Volt) strip of LEDs
  AddressableLEDBuffer m_buffer_5Volt = new AddressableLEDBuffer(100);

  //This is the first section of the 1 meter strip (currently out of 2)
  AddressableLEDBufferView m_1st_5V_section = m_buffer_12Volt.createView(0,49);

  //this will be used if we want the leds to be two
  AddressableLEDBufferView m_2nd_5V_section = m_buffer_12Volt.createView(50,99);


  public void setLedColor(double color) {
    
    blinkin.set(color);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setInitialColor() {

    setLedColor(Constants.Colors.Blue);
  }

  public void ChangeLedColor (int num) {

    switch (num) {
      case 1:  // lined_up
        blinkin.set(Colors.Green);
        break;
      case 2: // LimelightHelpers.getTV("limelight-two");
        blinkin.set(Colors.Red);
        break;
      case 3: // coralSensor
        blinkin.set(Colors.Lawn_Green);
        break;
      case 4: // algaeSensor
        blinkin.set(Colors.Blue_Violet);
        break;
      default:
        setInitialColor();
        break;
    }



  }




  public static final class Colors{
    //These are all the led optios, if you want more you will have to go to a REV website called "LED BLINKIN DRIVER"

    //Patterns:
    public static final double Rainbow_Rainbow_Pallet = -0.99;
    public static final double Rainbow_Ocean_Pallet = -0.95;
    public static final double Rainbow_Forest_Pallet = -0.91;
    public static final double Fire_Medium = -0.59;
    public static final double Fire_Large = -0.57;

    //Basic colors:
    public static final double Hot_Pink = 0.57;
    public static final double Dark_Red = 0.59;
    public static final double Red = 0.61;
    public static final double Red_Orange = 0.63;
    public static final double Orange = 0.65;
    public static final double Gold = 0.67;
    public static final double Yellow = 0.69;
    public static final double Lawn_Green = 0.71;
    public static final double Lime = 0.73;
    public static final double Dark_Green = 0.75;
    public static final double Green = 0.77;
    public static final double Blue_Green = 0.79;
    public static final double Aqua = 0.81;
    public static final double Sky_Blue = 0.83;
    public static final double Dark_Blue = 0.85;
    public static final double Blue = 0.87;
    public static final double Blue_Violet = 0.89;
    public static final double Violet = 0.91;
    public static final double White = 0.93;
    public static final double Gray = 0.95;
    public static final double Dark_Gray = 0.97;
    public static final double Black = 0.99;
  }

}