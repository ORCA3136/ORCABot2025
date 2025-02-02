// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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
        .inverted(false)
        .idleMode(IdleMode.kBrake);

      leftElevatorConfig
        .closedLoop
        // Set PID values for position control
        .p(0.1)
        .d(0.03)
        .outputRange(-1, 1)
        .maxMotion
        // Set MAXMotion parameters for position control
        .maxVelocity(3000)  // 4200
        .maxAcceleration(3000) // 6000
        .allowedClosedLoopError(0.5);


      rightElevatorConfig
        .closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
      rightElevatorConfig
        .idleMode(IdleMode.kBrake);
        

    }
  }

  public static final class WristConfigs {

    public static final SparkMaxConfig wristMotorConfig = new SparkMaxConfig();

    static {
      wristMotorConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake); // IDK about this; verify

      wristMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        // Set PID values for position control
        .p(0.01)
        //.i(0.02)
        .outputRange(-1, 1)
        .maxMotion
        // Set MAXMotion parameters for position control
        .maxVelocity(400)
        .maxAcceleration(600)
        .allowedClosedLoopError(0.5);
    }
  }

  public static final class SwerveDriveConfigs {
    
  }

  

  public static final class IntakeConfigs {
    public static final SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();

    static {
    intakeMotorConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    }
  }

}
