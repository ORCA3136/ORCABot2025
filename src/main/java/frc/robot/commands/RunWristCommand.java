// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;

import frc.robot.Constants;

// import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunWristCommand extends Command {
  
  private final ElevatorSubsystem elevatorSystem;
  private final double powerSetPoint;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunWristCommand(ElevatorSubsystem wristSystem, double power) {
    elevatorSystem = wristSystem;
    powerSetPoint = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSystem.setWristPower(powerSetPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSystem.setWristPower(0);
    // isFinished();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (powerSetPoint < 0 && elevatorSystem.getWristPosition() < Constants.Limits.kWristMinAngle)
        || (powerSetPoint > 0 && elevatorSystem.getWristPosition() > Constants.Limits.kWristMaxAngle);
  }
}
