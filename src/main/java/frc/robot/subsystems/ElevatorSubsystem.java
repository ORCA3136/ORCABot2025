// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpoints;
import frc.robot.Configs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class ElevatorSubsystem extends SubsystemBase {

  /** Instantiates elevator motors */
  SparkMax leftElevator = new SparkMax(Constants.SparkConstants.kLeftElevatorCanId, MotorType.kBrushless);
  SparkMax rightElevator = new SparkMax(Constants.SparkConstants.kRightElevatorCanId, MotorType.kBrushless);


  public enum Setpoint {
    kFeederStation,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4;
  }

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkClosedLoopController leftElevatorClosedLoopController =
      leftElevator.getClosedLoopController();
  private SparkClosedLoopController rightElevatorClosedLoopController =
      rightElevator.getClosedLoopController();
      
  private RelativeEncoder leftElevatorEncoder = leftElevator.getEncoder();  // only one encoder????????
  private RelativeEncoder rightElevatorEncoder = rightElevator.getEncoder();

  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private double elevatorCurrentTarget = Constants.ElevatorConstants.ElevatorSetpoints.kFeederStation;

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {
    // Zero elevator encoders on initialization
    // leftElevatorEncoder.setPosition(0);
    // rightElevatorEncoder.setPosition(0);
    // <l/r>ElevatorEncoder.setPosition(0);

    leftElevator.configure(Configs.ElevatorConfigs.leftElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightElevator.configure(Configs.ElevatorConfigs.rightElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  private void moveToSetpoint() {
    leftElevatorClosedLoopController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
    rightElevatorClosedLoopController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  /** Set the elevator motor power in the range of [-1, 1]. */
  private void setElevatorPower(double power) {
    leftElevator.set(power);
    rightElevator.set(power);
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kFeederStation:
              elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
              break;
            case kLevel1:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
              break;
            case kLevel2:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
              break;
            case kLevel3:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
              break;
            case kLevel4:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
              break;
          }
        });
  }


  /**
   * Command to run the elevator motor. When the command is interrupted, e.g. the button is released,
   * the motor will stop.
   */
  public Command ElevatorUpCommand() {
    return this.startEnd(
        () -> this.setElevatorPower(Constants.ElevatorConstants.ElevatorPowerLevels.kUp), () -> this.setElevatorPower(0.0));
  }

  /**
   * Command to reverses the elevator motor. When the command is interrupted, e.g. the button is
   * released, the motor will stop.
   */
  public Command ElevatorDownCommand() {
    return this.startEnd(
        () -> this.setElevatorPower(Constants.ElevatorConstants.ElevatorPowerLevels.kDown), () -> this.setElevatorPower(0.0));
  }

  public double getPos() {
    return rightElevatorEncoder.getPosition();
  }

  


  @Override
  public void periodic() {
    moveToSetpoint();
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
