// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Reef;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.commands.CappnCrunchCommand;
import frc.robot.commands.CenterLimelightOnReef;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.DoubleLidarRoutine;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.RunIntakeRoutine;
import frc.robot.commands.RunClimbSequenceCommand;
import frc.robot.commands.RunClimberCommand;
import frc.robot.commands.RunElevatorCommand;
import frc.robot.commands.RunFunnelCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunRecursiveIntakeRoutine;
import frc.robot.commands.RunIntakeScoreCommand;
import frc.robot.commands.RunVomitCommand;
import frc.robot.commands.RunWristCommand;
import frc.robot.commands.WaitForCoralCommand;
import frc.robot.commands.ZeroElevatorCommand;
import frc.robot.subsystems.ClimberSubsystemSim;
import frc.robot.subsystems.ElevatorSubsystemSim;
import frc.robot.subsystems.IntakeSubsystemSim;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ReefCentering;
import frc.robot.subsystems.SimMechanisms;
import frc.robot.subsystems.SwerveSubsystemSim;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private SwerveSubsystemSim driveBaseSim = new SwerveSubsystemSim(new File(Filesystem.getDeployDirectory(), "swerve/ORCA2025Sim"));
  private ElevatorSubsystemSim elevatorSim = new ElevatorSubsystemSim();
  private ClimberSubsystemSim climberSim = new ClimberSubsystemSim();
  private IntakeSubsystemSim intakeSim = new IntakeSubsystemSim();
  private SimMechanisms mechanismSim = new SimMechanisms(driveBaseSim, elevatorSim, climberSim, intakeSim);
  private ReefCentering reefCentering = new ReefCentering(driveBaseSim, elevatorSim);
  private final SendableChooser<Command> autoChooser;


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandJoystick m_secondaryController = new CommandJoystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure the trigger bindings
    configureBindings();
    configureNamedCommands();
    
    autoChooser = AutoBuilder.buildAutoChooser("default auto"); //pick a default
    SmartDashboard.putData("Auto Chooser", autoChooser);

    autoChooser.setDefaultOption("default auto", driveForwardAutoCommand);
    }
    
    
    SwerveInputStream driveAngularVelocity  = SwerveInputStream.of(driveBaseSim.getSwerveDrive(),
                                            () -> m_driverController.getLeftY(), 
                                            () -> m_driverController.getLeftX())
                                            .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
                                            .deadband(OperatorConstants.DEADBAND)
                                            .scaleTranslation(0.8)
                                            .allianceRelativeControl(true);

    SwerveInputStream driveRegular = driveAngularVelocity.copy().scaleTranslation(1.05);

    SwerveInputStream driveAngularVelocitySlow = driveAngularVelocity.copy().scaleTranslation(Constants.Limits.MEDIUM_SPEED_FACTOR);

  
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);
    SwerveInputStream driveRobotOrientedSlow = driveRobotOriented.copy().scaleTranslation(0.2);
    SwerveInputStream driveRobotOrientedFast  = driveRobotOriented.copy().scaleTranslation(2);

    Command driveFieldOrientedWithElevatorDampening = driveBaseSim.driveFieldOrientedElevatorSpeed(driveRegular, elevatorSim); // Normal drive with elevator dampening
    Command driveFieldOrientedAngularVelocitySlow = driveBaseSim.driveFieldOriented(driveAngularVelocitySlow); // Right stick
    Command driveRobotOrientedAngularVelocitySuperSlow = driveBaseSim.driveFieldOriented(driveRobotOrientedSlow); // Left stick
    Command driveRobotOrientedAngularVelocitySuperFast = driveBaseSim.driveFieldOriented(driveRobotOrientedFast);
    
    SwerveInputStream driveForwardAuto = SwerveInputStream.of(driveBaseSim.getSwerveDrive(),
                                            () -> 0.15, 
                                            () -> 0)
                                            .allianceRelativeControl(false);
    Command driveForwardAutoCommand = driveBaseSim.driveFieldOriented(driveForwardAuto);
    


  private void configureBindings() {
    driveBaseSim.setDefaultCommand(driveFieldOrientedWithElevatorDampening);
  
    // m_driverController.start().onTrue(Commands.runOnce(() -> driveBaseSim.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));

    m_driverController.button(1).onTrue(Commands.runOnce(() -> elevatorSim.setTargetSetpoint(ElevatorSubsystemSim.Setpoint.kFeederStation)));
    m_driverController.button(2).onTrue(Commands.runOnce(() -> elevatorSim.setTargetSetpoint(ElevatorSubsystemSim.Setpoint.kLevel2)));
    m_driverController.button(3).onTrue(Commands.runOnce(() -> elevatorSim.setTargetSetpoint(ElevatorSubsystemSim.Setpoint.kLevel3)));
    m_driverController.button(4).onTrue(Commands.runOnce(() -> elevatorSim.setTargetSetpoint(ElevatorSubsystemSim.Setpoint.kLevel4)));
  }

  private void configureNamedCommands() {
    /*
    NamedCommands.registerCommand("Intake score", new RunIntakeCommand(intake, Constants.IntakeConstants.IntakePowerLevels.kOut, vision,ledSubsystem).withTimeout(0.5));
    NamedCommands.registerCommand("Auto score", new AutoScoreCommand(intake, elevatorSystem, 0, vision));
    NamedCommands.registerCommand("Intake in", new DoubleLidarRoutine(intake, Constants.IntakeConstants.IntakePowerLevels.kFeed, vision, ledSubsystem, climber));
    NamedCommands.registerCommand("Wait for coral", new WaitForCoralCommand(vision));
    NamedCommands.registerCommand("Coral Centering", new DefaultIntakeCommand(intake, vision, climber, elevatorSystem));
    NamedCommands.registerCommand("Elevator L1", Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kFeederStation)));
    NamedCommands.registerCommand("Elevator L2", Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel2)));
    NamedCommands.registerCommand("Elevator L3", Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel3)));
    NamedCommands.registerCommand("Elevator L4", Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel4)));
    NamedCommands.registerCommand("Elevator Target L1", Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kFeederStation)));
    NamedCommands.registerCommand("Elevator Target L4", Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kLevel4)));
    NamedCommands.registerCommand("Elevator Bottom Algae", Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kBottomAlgae)));
    NamedCommands.registerCommand("Elevator Processor", Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kProcessor)));
    NamedCommands.registerCommand("Hold Algae", Commands.runOnce(() -> elevatorSystem.setElevatorPower(-0.5)).handleInterrupt(() -> elevatorSystem.setElevatorPower(0)));
    NamedCommands.registerCommand("Release Algae", Commands.runOnce(() -> elevatorSystem.setElevatorPower(0.5)).withTimeout(1).handleInterrupt(() -> elevatorSystem.setElevatorPower(0)));
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
