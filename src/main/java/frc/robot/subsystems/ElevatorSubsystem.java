// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Configs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class ElevatorSubsystem extends SubsystemBase {

  /** Instantiates elevator motors */
  SparkMax m_leftLifter = new SparkMax(Constants.SparkConstants.kLeftElevatorCanId, MotorType.kBrushless);
  SparkMax m_rightLifter = new SparkMax(Constants.SparkConstants.kRightElevatorCanId, MotorType.kBrushless);

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {

    m_leftLifter.configure(Configs.ElevatorConfigs.leftLifterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightLifter.configure(Configs.ElevatorConfigs.rightLifterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
