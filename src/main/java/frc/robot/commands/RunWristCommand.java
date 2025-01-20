// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.WristSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunWristCommand extends Command {
  
  private final WristSubsystem wristSubsystem;
  private final double powerSetPoint;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunWristCommand(WristSubsystem wristSubsystem, double power) {
    this.wristSubsystem = wristSubsystem;
    powerSetPoint = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristSubsystem.setWristPower(powerSetPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.setWristPower(0);
    wristSubsystem.setManuallyMoving(false);
    // isFinished();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
