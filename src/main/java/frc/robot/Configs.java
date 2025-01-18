// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * The Configs class ...
 */
public final class Configs {

  public static final class ElevatorConfigs {

    public static final SparkMaxConfig leftLifterConfig = new SparkMaxConfig();
    public static final SparkMaxConfig rightLifterConfig = new SparkMaxConfig();

    static {
      leftLifterConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake);

      rightLifterConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    }
  }

  public static final class SwerveDriveConfigs {

  }

}
