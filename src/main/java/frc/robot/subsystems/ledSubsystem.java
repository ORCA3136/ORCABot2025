// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;






public class ledSubsystem extends SubsystemBase {

  private AddressableLED m_led;

  

  public ledSubsystem() {}

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
//This should be the 5 meter strip of LEDs
AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(500);

// This is the view for the section of the strip on the left side of the robot.
// This section spans LEDs from index 0 through index 249, inclusive.
AddressableLEDBufferView m_left = m_buffer.createView(0, 249);

// This is the section of the strip on the right side of the robot.
// This section spans LEDs from index 250 through index 499, inclusive.
// The "reversed()" three lines down was on the example but probably isn't useful
AddressableLEDBufferView m_right = m_buffer.createView(250, 499)
/*.reversed()*/;

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

 // @Override
 // public void simulationPeriodic() {
 //   // This method will be called once per scheduler run during simulation
 // }
}

