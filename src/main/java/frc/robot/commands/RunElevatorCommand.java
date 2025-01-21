// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunElevatorCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final double powerSetPoint;
  private boolean safe = true;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunElevatorCommand(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, double power) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    powerSetPoint = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem, wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (wristSubsystem.isWristInTheWay()) {
      wristSubsystem.setSetpointCommand(WristSubsystem.Setpoint.unblock);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (wristSubsystem.isWristInTheWay()) {
      if (wristSubsystem.getCurrentTarget() != Constants.WristConstants.WristSetpoints.unblock) {
        wristSubsystem.setSetpointCommand(WristSubsystem.Setpoint.unblock);
      }
    } else if (!wristSubsystem.isManuallyMoving()) {
    elevatorSubsystem.setElevatorPower(powerSetPoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setElevatorPower(0);
    // isFinished();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
