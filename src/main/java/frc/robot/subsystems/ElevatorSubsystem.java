// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpoints;
import frc.robot.Configs;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
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

  private WristSubsystem wrist;
  private boolean blocking;

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkClosedLoopController elevatorClosedLoopController =
      leftElevator.getClosedLoopController();

  // private AbsoluteEncoder elevatorEncoder = leftElevator.getAbsoluteEncoder(); //Whichever motor has the encoder
  private RelativeEncoder elevatorEncoder = leftElevator.getEncoder();

  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private double elevatorCurrentTarget = Constants.ElevatorConstants.ElevatorSetpoints.kFeederStation;

  private boolean manuallyMoving = true;

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {
    // Zero elevator encoders on initialization
    // leftElevatorEncoder.setPosition(0);
    // rightElevatorEncoder.setPosition(0);
    // <l/r>ElevatorEncoder.setPosition(0);
    
    Configs.ElevatorConfigs.rightElevatorConfig
          .follow(leftElevator, false);
    
    leftElevator.configure(Configs.ElevatorConfigs.leftElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightElevator.configure(Configs.ElevatorConfigs.rightElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  private void moveToSetpoint() {
    elevatorClosedLoopController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  /** Set the elevator motor power in the range of [-1, 1]. */
  public void setElevatorPower(double power) {
    leftElevator.set(power);
    manuallyMoving = true;
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public void setSetpointCommand(Setpoint setpoint) {  // see wrist subsystem counterpart
    DataLogManager.log("setpoint command");
    //return this.runOnce(
        //() -> {
          setManuallyMoving(false);
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
  }
  

  public void setManuallyMoving(boolean bool) {
    manuallyMoving = bool;
  }
  
  public boolean isManuallyMoving() {
    return manuallyMoving;
  }
  

  public double getPos() {
    return elevatorEncoder.getPosition();
  }

  public void setWrist(WristSubsystem w) {
    wrist = w;
  }

  private void setBlocking(boolean b) {
    blocking = b;
  }

  public boolean elevatorInTheWay() {
    return blocking;
  }

  /*public boolean elevatorCanMove() {
    if (wrist.isWristInTheWay()) {
      return false;
    }
    return true;
  }

  public void makeElevatorMovable() {
    if (!elevatorCanMove()) {
      wrist.setSetpointCommand(WristSubsystem.Setpoint.unblock);
    }
  }  */


  @Override
  public void periodic() {
    //if () {

    //}
    if (!manuallyMoving) {
      if (!wrist.isWristInTheWay()) {
        moveToSetpoint();
      } else {
        wrist.setSetpointCommand(WristSubsystem.Setpoint.unblock);
      }
      
    }

    if (getPos() < Constants.ElevatorConstants.kElevatorSafetyThreshold) {
      //DataLogManager.log("Robot thinks the elevator is in the way");  // silly robit
      setBlocking(   false);      //true);
    } else {
      setBlocking(false);
    }

    SmartDashboard.putNumber("Elevator current target", elevatorCurrentTarget);
    SmartDashboard.putNumber("Elevator current position", getPos());
    SmartDashboard.putBoolean("Elevator manually moving", manuallyMoving);
    SmartDashboard.putBoolean("Elevator blocking status", elevatorInTheWay());
    
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
  
