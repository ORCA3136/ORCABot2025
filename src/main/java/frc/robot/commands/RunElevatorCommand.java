// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;




/** An example command that uses an example subsystem. */
public class RunElevatorCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem elevatorSubsystem;
  private final double powerSetPoint;
  //private boolean safe = true;

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
    if (elevatorSubsystem.wristInTheWay()) {
      /// =================================================================================================================
      /// elevatorSubsystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kUnblock); ///  EXTREMELY DANGEROUS BEFORE TUNING
      /// =================================================================================================================
    } else {
      elevatorSubsystem.setElevatorPower(powerSetPoint);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevatorSubsystem.wristInTheWay()) {

      if (elevatorSubsystem.getWristCurrentTarget() != Constants.WristConstants.WristSetpoints.unblock) {
        //elevatorSubsystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kUnblock);
      } else if (elevatorSubsystem.isWristManuallyMoving()) {
        elevatorSubsystem.setWristManuallyMoving(false);
      }

    } else if (!elevatorSubsystem.isWristManuallyMoving()) {
    elevatorSubsystem.setElevatorPower(powerSetPoint);
    }




    // //clear wrist if it is in the way and not already clearing in time
    // double eTarget;
    // double wTarget;
    // if (elevatorSubsystem.isManuallyMoving()) {
    //   eTarget = 90000 * elevatorSubsystem.getElevatorPower();
    //   wTarget = 90000 * elevatorSubsystem.getWristPower();
    // } else {
    //   eTarget = elevatorSubsystem.getElevatorCurrentTarget();
    //   wTarget = elevatorSubsystem.getWristCurrentTarget();
    // }

    // if ( (elevatorSubsystem.getElevatorPosition() < 7 && 7 < eTarget) || (elevatorSubsystem.getElevatorPosition() > 7 && 7 > eTarget) ) {
    //     if (elevatorSubsystem.getBottomWristX() > 9.75 || elevatorSubsystem.getHandX() > 9.75) {
    //       if (wTarget > Constants.WristConstants.WristSetpoints.unblock ) {
    //         // wait at the right pos
    //         elevatorSubsystem.setElevatorPower(0);
            
    //       } else {
    //       //clear wrist, go if timing is right
    //       elevatorSubsystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kUnblock);
    //       System.out.println("case1");
    //       }
    //     } else if (elevatorSubsystem.getWristPower() < 0 && wTarget < Constants.WristConstants.WristSetpoints.unblock) {
    //       //also clear wrist, but go
    //       elevatorSubsystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kUnblock);
    //       System.out.println("case2");
    //     }
    // } else {
    //   //don't touch wrist (except to protect the other bar)
    //   elevatorSubsystem.setElevatorPower(powerSetPoint);
    //     }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setElevatorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (powerSetPoint < 0 && elevatorSubsystem.getElevatorPosition() < Constants.Limits.kElevatorMinHeight)
        || (powerSetPoint > 0 && elevatorSubsystem.getElevatorPosition() > Constants.Limits.kElevatorMaxHeight);
  }
}