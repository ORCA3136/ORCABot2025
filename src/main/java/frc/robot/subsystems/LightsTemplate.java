// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class LightsTemplate extends SubsystemBase {
  /** Creates a new LightSubsystem. */

  Spark blinkin; 
 
  public LightsTemplate() {
 
    blinkin = new Spark(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void SetColor(double color) {

    blinkin.set(color);

  }
 
  public void ChangeColorOffRobotState(boolean Reading) {

    if (Reading == false) {

        blinkin.set(Constants.Colors.Lime);

    } else if (LimelightHelpers.getTX("limelight-ri") != 0) {

        blinkin.set(Constants.Colors.Red);

    } else {

        blinkin.set(Constants.Colors.Blue);

    }

   }
   
}