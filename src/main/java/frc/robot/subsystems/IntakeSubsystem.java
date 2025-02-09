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
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  SparkFlex intakeMotor = new SparkFlex(Constants.SparkConstants.kIntakeCanId, MotorType.kBrushless);

  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  private SparkClosedLoopController intakeMotorClosedLoopController =
      intakeMotor.getClosedLoopController();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor.configure(Configs.IntakeConfigs.intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** this one is pretty self explanitory*/
  public void setIntakePower(double power) {
    intakeMotor.set(power);
  }
  
  /**
   * Command to run the intake motor. When the command is interrupted, e.g. the button is released,
   * the motor will stop.
   */
  public Command intakeInCommand() {
    return this.startEnd(
        () -> this.setIntakePower(Constants.IntakeConstants.IntakePowerLevels.kIn), () -> this.setIntakePower(0.0));
  }

  public Command intakeOutCommand() {
    return this.startEnd(
        () -> this.setIntakePower(Constants.IntakeConstants.IntakePowerLevels.kOut), () -> this.setIntakePower(0.0));
  }

  public double getVelocity() {
    return intakeEncoder.getVelocity();
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
