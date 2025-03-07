// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LEDSubsystem;
import frc.robot.Constants.Colors;
import edu.wpi.first.wpilibj2.command.Command;

/** This command will eventually corospond with LEDSubsystem */
public class RunLEDsCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LEDSubsystem ledSubsystem;
  private double color;

  /**
   * Creates a new ExampleCommand.
   *
   * @param LEDSubsystem The subsystem used by this command.
   */
  public RunLEDsCommand(LEDSubsystem subsystem, double color) {
    ledSubsystem = subsystem;
    this.color = color;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledSubsystem.setLedColor(Colors.Blue);
    ledSubsystem.setInitialColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    ledSubsystem.setLedColor(Colors.Blue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
