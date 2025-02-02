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


    public static final class ElevatorSetpoints {
      public static final int kFeederStation = 0;
      public static final int kLevel1 = 0;
      public static final int kLevel2 = 14; // 0 -> 50
      public static final int kLevel3 = 50;
      public static final int kLevel4 = 85; // 92
    }
    public static final class ElevatorPowerLevels {
      public static final double kUp = 0.2;
      public static final double kDown = -0.1;
    }

  }

  public static final class WristConstants {

    

    public static final class WristSetpoints {
      public static final double unblock = 80;
      public static final double ks1 = 80;
      public static final double kLevel1 = 80; //2-> 80 temp names and values; change when identified
      public static final double kLevel4 = 80; // 35-> 80
    }

    public static final class WristPowerLevels {
      public static final double kUp = 0.10;
      public static final double kDown = -0.10;
    }
  }

  public static final class IntakeConstants {
    
    public static final class IntakePowerLevels {
      public static final double kIn = -0.40;
      public static final double kOut = 0.40;
    }
  }

  public static final class Limits {

    public static final double kElevatorSafetyThreshold = 5.0;

    public static final double kElevatorMaxHeight = 96;     //0.948;
    public static final double kElevatorMinHeight = 0.0;


    public static final double kWristSafetyThreshold = 80; //   -> -39


    public static final double kWristMinAngle = 1; // degrees
    public static final double kWristMaxAngle = 94; // degrees

    public static final double MAX_SPEED = Units.feetToMeters(6);  // theoretical: 14.63 Ft/s
  }


  public static final class PhysicalConstants {
    public static final double elevatorSupportBar = 34;
  }

  //some constants used for simulation.
  public static final class SimulationRobotConstants {
    public static final double kPixelsPerMeter = 20;

    public static final double kElevatorGearing = 20; // 20:1
    public static final double kCarriageMass =
        4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
    public static final double kElevatorDrumRadius = 0.04447 / 2.0; // m
    public static final double kMinElevatorHeightMeters = 0.9652; // m
    public static final double kMaxElevatorHeightMeters = 1.92; // m

    public static final double kArmReduction = 268; // 268:1
    public static final double kArmLength = 0.433; // m
    public static final double kArmMass = 4.3; // Kg
    public static final double kMinAngleRads =
        Units.degreesToRadians(-50.1); // -50.1 deg from horiz
    public static final double kMaxAngleRads =
        Units.degreesToRadians(40.9 + 180); // 40.9 deg from horiz

    public static final double kIntakeReduction = 135; // 135:1
    public static final double kIntakeLength = 0.4032262; // m
    public static final double kIntakeMass = 5.8738; // Kg
    public static final double kIntakeMinAngleRads = Units.degreesToRadians(80);
    public static final double kIntakeMaxAngleRads = Units.degreesToRadians(180);
    public static final double kIntakeShortBarLength = 0.1524;
    public static final double kIntakeLongBarLength = 0.3048;
    public static final double kIntakeBarAngleRads = Units.degreesToRadians(-60);
  }

  //This will hold all the LED options for the Blinkin
  public static final class Colors{
    public static final double Lime = 0.73;
    public static final double Red = 0.61;
    public static final double Blue = 0.87;
    
  }
 
}
