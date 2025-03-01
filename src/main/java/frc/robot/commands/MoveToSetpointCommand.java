// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** A command that moves the elevator and wrist to a position. */
public class MoveToSetpointCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem e_subsystem;
  private double y;
  private double theta;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveToSetpointCommand(ElevatorSubsystem subsystem, double y, double theta) {
    e_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    e_subsystem.wristMoveToSetpoint(Constants.Limits.kWristSafetyThreshold);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (e_subsystem.getWristAngle() +1 > Constants.Limits.kWristSafetyThreshold) {
      e_subsystem.elevatorMoveToSetpoint(y);
    } else if (e_subsystem.getElevatorPosition()+1 > y) {
      e_subsystem.wristMoveToSetpoint(theta);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (e_subsystem.getWristAngle() +1 > Constants.Limits.kWristSafetyThreshold && e_subsystem.getElevatorPosition()+1 > y);
  }
}
