// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;



public class SimMechanisms extends SubsystemBase {

  private ElevatorSubsystemYams elevatorSubsystem;
  private SwerveSubsystemSim swerveSubsystem;
  private ClimberSubsystemSim climberSubsystem;
  private IntakeSubsystemSim intakeSubsystem;

  StructPublisher<Pose2d> robotPosPublisher = NetworkTableInstance.getDefault()
    .getStructTopic("RobotPose", Pose2d.struct).publish();
  
  StructArrayPublisher<Pose3d> finalCompPosesPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("FinalComponentPoses", Pose3d.struct).publish();


  // Simulation display
  private final Mechanism2d elevatorMech = new Mechanism2d(400, 400);;
  private final MechanismRoot2d elevatorRoot = elevatorMech.getRoot("ElevatorRoot", 200, 50);
  private final MechanismLigament2d elevatorMechLig;

  // Visualization constants
  private final double ELEVATOR_VISUAL_WIDTH = 10.0; // Width of visualization in pixels
  private final double ELEVATOR_BASE_HEIGHT = 20.0; // Height of base in pixels
  private final double ELEVATOR_CARRIAGE_WIDTH = 30.0; // Width of elevator carriage in pixels
  // private final double ELEVATOR_CARRIAGE_HEIGHT = 40.0; // Height of elevator carriage in pixels
  private final double ELEVATOR_TOTAL_HEIGHT = ElevatorConstants.PhysicalConstants.maxHeightMeters - ElevatorConstants.PhysicalConstants.minHeightMeters;
  private final double ELEVATOR_VISUAL_SCALE_FACTOR = 300 / ELEVATOR_TOTAL_HEIGHT;




  /** Creates a new ExampleSubsystem. */
  public SimMechanisms(SwerveSubsystemSim swerve, ElevatorSubsystemYams elevator, ClimberSubsystemSim climber, IntakeSubsystemSim intake) {
    swerveSubsystem = swerve;
    elevatorSubsystem = elevator;
    climberSubsystem = climber;
    intakeSubsystem = intake;

    /* Create a 2d visual of the elevator */
    /* Add elevator base */
    MechanismLigament2d elevatorBase = elevatorRoot.append(
      new MechanismLigament2d(
        "Base",
        ELEVATOR_BASE_HEIGHT,
        90,
        6,
        new Color8Bit(Color.kDarkGray)
      ));
    /* Add elevator tower */
    MechanismLigament2d elevatorTower = elevatorBase.append(
      new MechanismLigament2d(
        "Tower",
        ELEVATOR_TOTAL_HEIGHT * ELEVATOR_VISUAL_SCALE_FACTOR,
        90,
        ELEVATOR_VISUAL_WIDTH,
        new Color8Bit(Color.kGray)
      ));
    /* Add elevator carriage */
    elevatorMechLig = elevatorRoot.append(
      new MechanismLigament2d(
        "Elevator",
        ELEVATOR_BASE_HEIGHT,
        90,
        ELEVATOR_CARRIAGE_WIDTH,
        new Color8Bit(Color.kBlue)
      ));





    /* Initialize visualization */
    SmartDashboard.putData("Elevator Sim", elevatorMech);
  }

  @Override
  public void periodic() { 
    // Update elevator height
    double currentHeight = elevatorSubsystem.getElevatorSimulation().getPositionMeters();
    double displayHeight = ELEVATOR_BASE_HEIGHT + (currentHeight - ElevatorConstants.PhysicalConstants.minHeightMeters) * ELEVATOR_VISUAL_SCALE_FACTOR;
    elevatorMechLig.setLength(displayHeight);

    // Add telemetry data
    SmartDashboard.putNumber("Elevator Height (m)", currentHeight);
    SmartDashboard.putNumber(
      "Elevator Velocity (m/s)",
      elevatorSubsystem.getElevatorSimulation().getVelocityMetersPerSecond()
    );
    SmartDashboard.putNumber(
      "Elevator Current (A)",
      elevatorSubsystem.getElevatorSimulation().getCurrentDrawAmps()
    );
  }

  @Override
  public void simulationPeriodic() {
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSubsystem.getSimCurrent() + 
                                                        swerveSubsystem.getCurrentDraw() + 
                                                        climberSubsystem.getCurrentDraw()));

    robotPosPublisher.set(swerveSubsystem.getPose());
    
    // Updating the components of the AdvantageScope 3d Model
    finalCompPosesPublisher.set(new Pose3d[]
      {
        // First stage elevator, second stage elevator, wrist, funnel, climber, foot
        new Pose3d(-0.1, 0, 0.1 + elevatorSubsystem.getElevatorPositionMeters(), new Rotation3d()),
        new Pose3d(-0.1, 0, 0.115 + 1.7 * elevatorSubsystem.getElevatorPositionMeters(), new Rotation3d()),
        new Pose3d(-0.266, 0, 0.436 + 1.7 * elevatorSubsystem.getElevatorPositionMeters(), 
                  new Rotation3d(0, - elevatorSubsystem.getWristPositionRadians(), 0)),
        new Pose3d(-0.0215, 0, 0.963, new Rotation3d(0, -climberSubsystem.getFunnelAngleRadians(), 0)),
        new Pose3d(0.336, 0, 0.399, new Rotation3d(0, -climberSubsystem.getClimberAngleRadians(), 0)),
        new Pose3d(0.3435, 0, 0.128, new Rotation3d(0, 0, 0))
      }
    );
  }
}
