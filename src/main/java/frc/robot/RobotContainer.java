// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // private final SwerveSubsystem driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/ORCA2025"));
  private final SwerveSubsystem driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/ORCA2025"));


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

  }

  SwerveInputStream driveAngularVelocity  = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                            () -> -m_driverController.getLeftY(), 
                                            () -> -m_driverController.getLeftX())
                                            .withControllerRotationAxis(() -> -m_driverController.getRightX())
                                            .deadband(0.1)
                                            .scaleTranslation(0.8)
                                            .allianceRelativeControl(true);
  Command driveFieldOrientedAngularVelocity = driveBase.driveFieldOriented(driveAngularVelocity);

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driveBase.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
