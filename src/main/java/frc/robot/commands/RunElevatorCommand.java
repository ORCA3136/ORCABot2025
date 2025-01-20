// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunElevatorCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem elevatorSubsystem;
  private final double powerSetPoint;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunElevatorCommand(ElevatorSubsystem elevatorSubsystem, double power) {
    this.elevatorSubsystem = elevatorSubsystem;
    powerSetPoint = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setElevatorPower(powerSetPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setElevatorPower(0);
    elevatorSubsystem.setManuallyMoving(false);
    // isFinished();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
