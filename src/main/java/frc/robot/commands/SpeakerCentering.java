package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;


public class SpeakerCentering extends Command {

  IntakeSubsystem m_IntakeSubsystem;
  SwerveSubsystem m_DriveSubsystem;
  VisionSubsystem m_SensorSubsystem;
  CommandXboxController m_controller;

  // Angle to speaker
  // Distance to speaker
  // Current speed for offset

  double shooterTarget;
  double armTarget;

  double shooterSpeed;
  double armPosition;

  boolean startedShot = false;
  boolean finished = false;

  public SpeakerCentering(VisionSubsystem visionSubsystem,
        SwerveSubsystem SwerveSubsystem, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_SensorSubsystem = visionSubsystem;
    m_DriveSubsystem = SwerveSubsystem;
    m_controller = controller;

    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startedShot = false;
    finished = false;
    // DataLogManager.log("Auto shooting - init");

    m_DriveSubsystem.speakerCentering(m_controller, m_SensorSubsystem).schedule();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // m_DriveSubsystem.regularDrive(m_controller).schedule();

    // DataLogManager.log("Auto shooting --------- end");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}