// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  //Creates a new subsystem for the LEDs

  Spark blinkin;

  private AddressableLED m_led;

  public LEDSubsystem() {

    blinkin = new spark(0)
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  
  // //These Adressable LED buffers are probably not going to be used, but I will leave them here just in case we want to divide the LEDs into multiple sections
  // //Some of these lines are double comented so that you can highlite the entire thing and Ctrl //
  // //This should be the 5 meter (12 Volt) strip of LEDs
  // AddressableLEDBuffer m_buffer_12Volt = new AddressableLEDBuffer(500);

  // // This is the view for the section of the strip on the left side of the robot.
  // // This section spans LEDs from index 0 through index 249, inclusive.
  // AddressableLEDBufferView m_1st_12V_section = m_buffer.createView(0, 499);

  // // This is the section of the strip on the right side of the robot.
  // // This section spans LEDs from index 250 through index 499, inclusive.
  // // The "reversed()" three lines down was on the example but probably isn't useful
  // //AddressableLEDBufferView m_2nd_12V_section = m_buffer.createView(250, 499);
  // /*.reversed()*/

  // //This should help control the 1 meter (5 Volt) strip of LEDs
  // AddressableLEDBuffer m_buffer_5Volt = new AddressableLEDBuffer(100);

  // //This is the first section of the 1 meter strip (currently out of 2)
  // AddressableLEDBufferView m_1st_5V_section = m_buffer.createView(0,99);

  // //this will be used if we want the leds to be two
  // //AddressableLEDBufferView m_2nd_5V_section = m_buffer.createView(50,99);


  public void setLedColor(Blue) {
    
    return false;
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

 //This is commented out because we probably won't use the LEDs durring simulation
 // @Override
 // public void simulationPeriodic() {
 //   // This method will be called once per scheduler run during simulation
 // }
}

