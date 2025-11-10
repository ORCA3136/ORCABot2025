// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class IntakeSubsystemSim extends SubsystemBase {

  SparkFlex intakeMotor = new SparkFlex(Constants.SparkConstants.kIntakeCanId, MotorType.kBrushless);

  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
 
  private static final SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();

  static {
  intakeMotorConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake);
  }


  private final DCMotor m_intakeGearbox = DCMotor.getNEO(1);
  private double intakeGearing = 50;

  // private final LinearSystemSim m_linearSim = new LinearSystemSim<>(null, 0);

  // private final FlywheelSim m_intakeSim = 
  //     new FlywheelSim(null, m_intakeGearbox, 0);

  private final SparkFlexSim m_intakeMotorSim = new SparkFlexSim(intakeMotor, m_intakeGearbox);
  private final SparkRelativeEncoderSim m_intakeEncoderSim = m_intakeMotorSim.getRelativeEncoderSim();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystemSim() {
    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** this one is pretty self explanitory*/
  public void setIntakePower(double power) {
    intakeMotor.set(power);
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
