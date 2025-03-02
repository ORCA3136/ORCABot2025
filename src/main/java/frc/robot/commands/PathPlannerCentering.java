// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** A command that centers the robot with the reef. */
public class PathPlannerCentering extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem m_Drive;

  //private Field field = 



  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PathPlannerCentering(SwerveSubsystem subsystem) {
    m_Drive = subsystem;
    addRequirements(subsystem);
  }

  public Pose2d nearestPose2d(Pose2d currentPose2d) {
    return currentPose2d.nearest(Constants.REEF_POSE2DLIST);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
