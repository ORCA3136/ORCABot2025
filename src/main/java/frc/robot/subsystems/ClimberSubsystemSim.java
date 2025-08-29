// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class ClimberSubsystemSim extends SubsystemBase {

  SparkMax climberMotor = new SparkMax(Constants.SparkConstants.kClimberCanId, MotorType.kBrushless);
  SparkMax funnelMotor = new SparkMax(Constants.SparkConstants.kFunnelCanId, MotorType.kBrushless);

  private RelativeEncoder climberEncoder = climberMotor.getEncoder();
  private RelativeEncoder funnelEncoder = funnelMotor.getEncoder();

  private boolean startedClimbing = false;


  private final DCMotor m_climberGearbox = DCMotor.getNEO(1);
  private final DCMotor m_funnelGearbox = DCMotor.getNEO(1);
  private double climberGearing = 50;
  private double funnelGearing = 50;

  private final SingleJointedArmSim m_climberSim = 
      new SingleJointedArmSim(
        m_climberGearbox, 
        climberGearing, 
        10, 
        0.3, 
        -30, 
        30, 
        false, 
        0, 
        0,
        0);
    private final SingleJointedArmSim m_funnelSim = 
        new SingleJointedArmSim(
          m_funnelGearbox, 
          funnelGearing, 
          10, 
          0.3, 
          0, 
          30, 
          false, 
          0, 
          0,
          0);

  private final SparkMaxSim m_climberMotorSim = new SparkMaxSim(climberMotor, m_climberGearbox);
  private final SparkRelativeEncoderSim m_climberEncoderSim = m_climberMotorSim.getRelativeEncoderSim();

  private final SparkMaxSim m_funnelMotorSim = new SparkMaxSim(funnelMotor, m_funnelGearbox);
  private final SparkRelativeEncoderSim m_funnelEncoderSim = m_funnelMotorSim.getRelativeEncoderSim();


  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystemSim() {
    funnelMotor.configure(Configs.ClimberConfigs.funnelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climberMotor.configure(Configs.ClimberConfigs.climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  
  public void setClimberPower(double power) {
    climberMotor.set(power);
  }

  public void setFunnelPower(double power) {
    funnelMotor.set(power);
  }

  public double getFunnelPosition() {
    return funnelEncoder.getPosition();
  }

  public double getClimberPosition() {
    return climberEncoder.getPosition();
  }

  public boolean isFlipped() {
    return getFunnelPosition() < Constants.ClimberConstants.kFunnelOutPos;
  }

  public void setClimbingMode(boolean mode) {
    startedClimbing = mode;
  }

  public boolean getClimbingMode() {
    return startedClimbing;
  }

  public double getClimberAngleRadians() {
    return m_climberSim.getAngleRads();
  }

  public double getFunnelAngleRadians() {
    return m_funnelSim.getAngleRads();
  }

  public double getCurrentDraw() {
    return m_climberSim.getCurrentDrawAmps() + m_funnelSim.getCurrentDrawAmps();
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("funnel position", getFunnelPosition());
    SmartDashboard.putNumber("climber Position", getClimberPosition());
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_climberSim.setInput(climberMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_funnelSim.setInput(funnelMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_climberSim.update(0.020);
    m_funnelSim.update(0.020);

    // Encoder and motor positions do not match
    m_climberMotorSim.iterate(
      Units.radiansPerSecondToRotationsPerMinute(
        m_climberSim.getVelocityRadPerSec() * climberGearing),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        0.02); // Time interval, in Seconds
    m_funnelMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(
          m_funnelSim.getVelocityRadPerSec() * funnelGearing),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        0.02); // Time interval, in Seconds
  }
}
