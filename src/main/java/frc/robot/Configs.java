// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * The Configs class ...
 */
public final class Configs {

  public static final class ElevatorConfigs {

    public static final SparkMaxConfig leftElevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig rightElevatorConfig = new SparkMaxConfig();

    static {
      leftElevatorConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50)
        .voltageCompensation(12);

      leftElevatorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control
        .pidf(1, 0.0, 2, 0.3)
        .iMaxAccum(0)
        .outputRange(-0.8, 1);


      // rightElevatorConfig
      //   .closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
      rightElevatorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50)
        .voltageCompensation(12);
    }
  }

  public static final class WristConfigs {

    public static final SparkMaxConfig wristMotorConfig = new SparkMaxConfig();

    static {
      wristMotorConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(15);

      wristMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        // Set PID values for position control
        .pid(5, 0.0, 4)
        .outputRange(-1, 1);
    }
  }

  public static final class IntakeConfigs {
    public static final SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();

    static {
    intakeMotorConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    }
  }

  public static final class ClimberConfigs {
    public static final SparkFlexConfig funnelMotorConfig = new SparkFlexConfig();

    static {
      funnelMotorConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast);
      }

    public static final SparkFlexConfig climberMotorConfig = new SparkFlexConfig();

    static {
      climberMotorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);
      }
  }

}
