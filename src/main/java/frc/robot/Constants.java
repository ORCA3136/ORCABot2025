// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.node.DoubleNode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.05;
    public static final double kTriggerDeadband = 0.05;
  }

  public static final class SparkConstants {
    // SPARK MAX CAN IDs

    // Drive Neos
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kRearRightDrivingCanId = 8;

    //Drive 550s
    public static final int kFrontLeftTurningCanId = 1;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearLeftTurningCanId = 5;
    public static final int kRearRightTurningCanId = 7;

    // Elevator 
    public static final int kLeftElevatorCanId = 10;
    public static final int kRightElevatorCanId = 11;

    // Wrist
    public static final int kWristCanId = 20;

    // Intake
    public static final int kIntakeCanId = 21;


    // // Elevator 
    // public static final int kLeftElevatorCanId = 12;
    // public static final int kRightElevatorCanId = 13;

    // // Wrist
    // public static final int kWristCanId = 14;

    // // Intake
    // public static final int kIntakeCanId = 15;


    public static final boolean kGyroReversed = false;

  }

  public static final class DriveConstants {


    // Chassis configuration
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

  }

  public static final class ElevatorConstants {

    public static final double kElevatorSafetyThreshold = 12.0;

    public static final class ElevatorSetpoints {
      public static final int kFeederStation = 0;
      public static final int kLevel1 = 0;
      public static final int kLevel2 = 50; // 0 -> 50
      public static final int kLevel3 = 100;
      public static final int kLevel4 = 150;
    }
    public static final class ElevatorPowerLevels {
      public static final double kUp = 0.201;
      public static final double kDown = -0.101;
    }

  }

  public static final class WristConstants {

    public static final double kWristSafetyThreshold = 0.35;

    public static final class WristSetpoints {
      public static final double unblock = 10;
      public static final double ks1 = -3;
      public static final double ks2 = 0; // temp names and values; change when identified
      public static final double ks3 = 3;
    }

    public static final class WristPowerLevels {
      public static final double kUp = 0.101;
      public static final double kDown = -0.101;
    }
  }

  public static final class IntakeConstants {
    
    public static final class IntakePowerLevels {
      public static final double kIn = -0.2;
      public static final double kOut = 0.2;
    }
  }

  public static final class Limits {
    public static final double kElevatorMaxHeight = 50;
    public static final double kElevatorMinHeight = 10;
  }

  public static final double MAX_SPEED = Units.feetToMeters(4.5);

}
