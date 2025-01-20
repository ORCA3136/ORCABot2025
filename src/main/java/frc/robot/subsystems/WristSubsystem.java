// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpoints;

import com.revrobotics.spark.SparkBase.ControlType;

public class WristSubsystem extends SubsystemBase {
  
  SparkMax wristMotor = new SparkMax(Constants.SparkConstants.kWristCanId, MotorType.kBrushless);

  public enum Setpoint {
    ks1, // KSI??!!?!
    ks2, // see note in constants file
    ks3,
  }

  // Initialize wrist SPARK. We will use MAXMotion position control for the wrist, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkClosedLoopController wristClosedLoopController =
      wristMotor.getClosedLoopController();

  // private AbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder();
  private RelativeEncoder wristEncoder = wristMotor.getEncoder();

  private double wristCurrentTarget = Constants.WristConstants.WristSetpoints.ks1; // ksI - into the thick of it?
  private boolean manuallyMoving = false;

  /** Creates a new WristSubsystem. */
  public WristSubsystem() {
    
    wristMotor.configure(Configs.WristConfigs.wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  private void moveToSetpoint() {
    wristClosedLoopController.setReference(
        wristCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  public void setWristPower(double power) {
    wristMotor.set(power);
    setManuallyMoving(true);
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          setManuallyMoving(false);
          switch (setpoint) {
            case ks1:
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.ks1;
              break;
            case ks2:
            wristCurrentTarget = Constants.WristConstants.WristSetpoints.ks2;
              break;
            case ks3:
            wristCurrentTarget = Constants.WristConstants.WristSetpoints.ks3;
              break;
          }
        });
  }

  /**
   * Command to run the wrist motor. When the command is interrupted, e.g. the button is released,
   * the motor will stop.
   */
  public Command WristForwardCommand() {
    return this.startEnd(
        () -> this.setWristPower(Constants.WristConstants.WristPowerLevels.kUp), () -> this.setWristPower(0.0));
  }

  /**
   * Command to reverses the elevator motor. When the command is interrupted, e.g. the button is
   * released, the motor will stop.
   */
  public Command WristBackCommand() {
    return this.startEnd(
        () -> this.setWristPower(Constants.WristConstants.WristPowerLevels.kDown), () -> this.setWristPower(0.0));
  }

  public double getPos() {
    return wristEncoder.getPosition();
  }

  public void setManuallyMoving(boolean b) {
    manuallyMoving = b;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!manuallyMoving)
      moveToSetpoint();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
