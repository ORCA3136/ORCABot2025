// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;




/** An example command that uses an example subsystem. */
public class UppercutCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intake;
  //private boolean safe = true;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public UppercutCommand(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intake) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.intake = intake;
    
    addRequirements(elevatorSubsystem, intake);
  }
  // Constants.Limits.kElevatorMaxHeight

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.wristMoveToSetpoint(100); // Constants.WristConstants.WristSetpoints.kAlgae1 //135
    intake.setIntakePower(Constants.IntakeConstants.IntakePowerLevels.kAlgaeHold);
    // elevatorSubsystem.setMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevatorSubsystem.getWristPosition() > 25) {
      elevatorSubsystem.setElevatorManuallyMoving(true);
      elevatorSubsystem.setElevatorPower(0.9);
    }
    if (elevatorSubsystem.getElevatorPosition() > 45) {
      elevatorSubsystem.wristMoveToSetpoint(10);
      intake.setIntakePower(Constants.IntakeConstants.IntakePowerLevels.kAlgaeOut);
    }
    if (elevatorSubsystem.getElevatorPosition() > 58) {
      elevatorSubsystem.setElevatorPower(-0.1);
      
      elevatorSubsystem.setElevatorPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setElevatorPower(0);
    intake.setIntakePower(Constants.IntakeConstants.IntakePowerLevels.kAlgaeOut);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (elevatorSubsystem.getElevatorPosition() >= Constants.Limits.kElevatorMaxHeight-5) && (elevatorSubsystem.getWristPosition() < 15);
  }
}