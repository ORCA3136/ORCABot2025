// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class SimMechanisms extends SubsystemBase {

  private ElevatorSubsystemSim elevatorSubsystemSim;

  StructPublisher<Pose2d> robotPosPublisher = NetworkTableInstance.getDefault()
    .getStructTopic("RobotPose", Pose2d.struct).publish();
  
  StructPublisher<Pose3d> finalCompPosesPub_3 = NetworkTableInstance.getDefault()
    .getStructTopic("FinalComponentPoses_3", Pose3d.struct).publish();
  StructPublisher<Pose3d> finalCompPosesPub_4 = NetworkTableInstance.getDefault()
    .getStructTopic("FinalComponentPoses_4", Pose3d.struct).publish();
  StructPublisher<Pose3d> finalCompPosesPub_5 = NetworkTableInstance.getDefault()
    .getStructTopic("FinalComponentPoses_5", Pose3d.struct).publish();

  /** Creates a new ExampleSubsystem. */
  public SimMechanisms(ElevatorSubsystemSim elevator) {
    elevatorSubsystemSim = elevator;


    m_mech3rdRootTop.append(new MechanismLigament2d("Elevator3rdTop", 1, 0));
    m_mech3rdRootTop.append(new MechanismLigament2d("Elevator3rdLeft", 1, 270));
    m_mech3rdRootBottom.append(new MechanismLigament2d("Elevator3rdBottom", 1, 180));
    m_mech3rdRootBottom.append(new MechanismLigament2d("Elevator3rdRight", 1, 90));
  }


  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_elevatorMechanism = new Mechanism2d(4, 5);
  private final MechanismRoot2d m_mech3rdRootTop = m_elevatorMechanism.getRoot("Root 3rdTop", 1, 1);
  private final MechanismRoot2d m_mech3rdRootBottom = m_elevatorMechanism.getRoot("Root 3rdBot", 2, 0);
  

  private void updateTelemetry(double height) {
    m_mech3rdRootTop.setPosition(1, 1 + height);
    m_mech3rdRootBottom.setPosition(2, 0 + height);
  }

  @Override
  public void periodic() { }

  @Override
  public void simulationPeriodic() {
    robotPosPublisher.set(new Pose2d(0, 0, new Rotation2d()));
    
    finalCompPosesPub_3.set(new Pose3d(
      -0.0215, 0, 0.963, new Rotation3d(0, 0 * Math.sin(Timer.getFPGATimestamp()), 0)));
    finalCompPosesPub_4.set(new Pose3d(
      0.336, 0, 0.399, new Rotation3d(0, 0 * Math.sin(Timer.getFPGATimestamp()), 0)));
    finalCompPosesPub_5.set(new Pose3d(
      0.3435, 0, 0.128, new Rotation3d(0, 0 * Math.sin(Timer.getFPGATimestamp()), 0)));

    // SmartDashboard.putNumber("Elevator Height", elevatorSubsystemSim.getElevatorPositionMeters());
    // updateTelemetry(elevatorSubsystemSim.getElevatorPositionMeters());

    // SmartDashboard.putData("Elevator Mechanism", m_elevatorMechanism);
  }
}
