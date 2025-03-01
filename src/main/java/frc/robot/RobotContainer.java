// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AprilTagFollowCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RunClimberCommand;
import frc.robot.commands.RunElevator;
import frc.robot.commands.RunElevatorCommand;
import frc.robot.commands.RunFunnelCommand;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunIntakeRoutine;
import frc.robot.commands.RunWrist;
import frc.robot.commands.RunWristCommand;
import frc.robot.commands.ZeroElevatorCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private VisionSubsystem vision = new VisionSubsystem();
  private final SwerveSubsystem driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/ORCA2025"), vision); // where to configure the robot or "choose" it
  private IntakeSubsystem intake = new IntakeSubsystem();
  private ElevatorSubsystem elevatorSystem = new ElevatorSubsystem();
  private ClimberSubsystem climber = new ClimberSubsystem();
  private LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final SendableChooser<Command> autoChooser;


  

  // BooleanLogEntry myBooleanLog;
  // DoubleLogEntry myDoubleLog;
  // StringLogEntry myStringLog;
  


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
    
    autoChooser = AutoBuilder.buildAutoChooser("My Default Auto"); //pick a default
    SmartDashboard.putData("Auto Chooser", autoChooser);

    
    }
    
      SwerveInputStream driveAngularVelocity  = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                            () -> -m_driverController.getLeftY(), 
                                            () -> -m_driverController.getLeftX())
                                            .withControllerRotationAxis(() -> -m_driverController.getRightX())
                                            .deadband(OperatorConstants.DEADBAND)
                                            .scaleTranslation(0.8)
                                            .allianceRelativeControl(true);

  
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                                          .withControllerHeadingAxis(m_driverController::getRightX, 
                                                          m_driverController::getRightY).headingWhile(false);
                                                         //withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightX  <- change this to Y for special mode
  Command driveFieldOrientedDirectAngle = driveBase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAngularVelocity = driveBase.driveFieldOriented(driveAngularVelocity);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                              () -> m_driverController.getLeftY(),
                                              () -> m_driverController.getLeftX())
                                              .withControllerRotationAxis(m_driverController::getRightX)
                                              .deadband(OperatorConstants.DEADBAND)
                                              .scaleTranslation(0.8)
                                              .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    m_driverController.getRawAxis(
                                                                                                        4) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    m_driverController.getRawAxis(
                                                                                                        4) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = driveBase.driveFieldOriented(driveDirectAngleSim);


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
    // Yagsl - driveFieldOrientedAngularVelocity
    // us    - driveFieldOrientedDirectAngle

    // driveBase.driveTankDrive();

    // if (RobotBase.isSimulation())
    // {
    //   driveBase.setDefaultCommand(RobotBase.isSimulation() ? 
    //                             driveFieldOrientedDirectAngle :
    //                             driveFieldOrientedDirectAngleSim);
    // }
    // if (Robot.isSimulation())
    // {
    //   m_driverController.start().onTrue(Commands.runOnce(() -> driveBase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    // }
    // else
    // {
      m_driverController.start().onTrue(Commands.runOnce(driveBase::zeroGyro));


      // m_driverController.leftBumper().whileTrue(new RunElevatorCommand(elevator, wrist, Constants.ElevatorConstants.ElevatorPowerLevels.kUp));
      // m_driverController.rightBumper().whileTrue(new RunElevatorCommand(elevator, wrist, Constants.ElevatorConstants.ElevatorPowerLevels.kDown));

      //m_driverController.leftBumper().onTrue(Commands.runOnce(  () -> {elevator.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel4);       wrist.setSetpointCommand(WristSubsystem.Setpoint.ks3);}));
      //m_driverController.rightBumper().onTrue(Commands.runOnce( () -> {elevator.setSetpointCommand(ElevatorSubsystem.Setpoint.kFeederStation);wrist.setSetpointCommand(WristSubsystem.Setpoint.ks1);}));

      // m_driverController.rightBumper().whileTrue(new RunWristCommand(elevatorSystem, Constants.WristConstants.WristPowerLevels.kUp));
      // m_driverController.leftBumper().whileTrue(new RunWristCommand(elevatorSystem, Constants.WristConstants.WristPowerLevels.kDown));

      m_driverController.y().whileTrue(new RunElevator(elevatorSystem, Constants.ElevatorConstants.ElevatorPowerLevels.kUp));
      m_driverController.a().whileTrue(new RunElevator(elevatorSystem, Constants.ElevatorConstants.ElevatorPowerLevels.kDown));
      m_driverController.b().whileTrue(new RunWristCommand(elevatorSystem, Constants.WristConstants.WristPowerLevels.kUp));
      m_driverController.x().whileTrue(new RunWristCommand(elevatorSystem, Constants.WristConstants.WristPowerLevels.kDown));

      m_driverController.povDown().whileTrue(driveBase.driveToPoseRobotRelative(new Pose2d(-1, 0, new Rotation2d(0))));
      m_driverController.povUp().whileTrue(driveBase.driveToPoseRobotRelative(new Pose2d(1, 0, new Rotation2d(0))));
      m_driverController.povLeft().whileTrue(driveBase.driveToPoseRobotRelative(new Pose2d(0, 0, new Rotation2d(Math.PI/4))));
      m_driverController.povRight().whileTrue(driveBase.driveToPoseRobotRelative(new Pose2d(0, 0, new Rotation2d(-Math.PI/4))));

      // m_driverController.povDown().whileTrue(Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel1)));
      // m_driverController.povLeft().whileTrue(Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel2)));
      // m_driverController.povRight().whileTrue(Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel3)));
      // m_driverController.povUp().whileTrue(Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel4)));


      // m_driverController.axisGreaterThan(3, 0.5).whileTrue(new RunWristCommand(wrist, Constants.WristConstants.WristPowerLevels.kDown));  // was used in testing
      // m_driverController.axisGreaterThan(2, 0.5).whileTrue(new RunWristCommand(wrist, Constants.WristConstants.WristPowerLevels.kUp));

      m_driverController.axisGreaterThan(3, 0.4).whileTrue(new RunIntakeCommand(intake, Constants.IntakeConstants.IntakePowerLevels.kOut, vision));
      m_driverController.axisGreaterThan(2, 0.4).whileTrue(new RunIntakeCommand(intake, Constants.IntakeConstants.IntakePowerLevels.kIn, vision));
      m_driverController.rightBumper().whileTrue(new RunIntakeRoutine(intake, Constants.IntakeConstants.IntakePowerLevels.kFeed, vision));
      m_driverController.leftBumper().onTrue(Commands.runOnce(() -> driveBase.setSpeed(Constants.Limits.MEDIUM_SPEED))).onFalse(Commands.runOnce(() -> driveBase.setSpeed(Constants.Limits.MAX_SPEED)));
      // m_driverController.leftBumper().onFalse(Commands.runOnce(() -> driveBase.setSpeed(Constants.Limits.MEDIUM_SPEED))).onTrue(Commands.runOnce(() -> driveBase.setSpeed(Constants.Limits.MAX_SPEED)));

      m_secondaryController.button(11).whileTrue(new RunClimberCommand(climber, Constants.ClimberConstants.kClimberIn));
      m_secondaryController.button(10).whileTrue(new RunClimberCommand(climber, Constants.ClimberConstants.kClimberOut));
      m_secondaryController.button(9).whileTrue(new RunFunnelCommand(climber, Constants.ClimberConstants.kFunnelSpeed));
      m_secondaryController.button(12).whileTrue(driveBase.driveToPoseRobotRelative(new Pose2d(new Translation2d(1, 0), Rotation2d.fromDegrees(0))));

      m_secondaryController.button(1).whileTrue(Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel4)));
      m_secondaryController.button(2).whileTrue(Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel3)));
      m_secondaryController.button(3).whileTrue(Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel2)));
      m_secondaryController.button(4).whileTrue(Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel1)));

      m_secondaryController.button(8).whileTrue(Commands.runOnce(() -> driveBase.resetOdometry(new Pose2d())));
      m_secondaryController.button(7).whileTrue(driveBase.driveToPose(new Pose2d(new Translation2d(4.937, 4.871), Rotation2d.fromDegrees(-116))));
      m_secondaryController.button(6).whileTrue(Commands.runOnce(() ->elevatorSystem.setWristTarget(67)));
      m_secondaryController.button(5).whileTrue(Commands.runOnce(() -> ledSubsystem.setLedColor(Constants.Colors.Gold)));

      // m_driverController.leftBumper().whileTrue(new AprilTagFollowCommand(driveBase));

      // m_driverController.axisGreaterThan(3, 0.4).whileTrue(new RunIntake(intake, Constants.IntakeConstants.IntakePowerLevels.kIn));
      // m_driverController.axisGreaterThan(2, 0.4).whileTrue(new RunIntake(intake, Constants.IntakeConstants.IntakePowerLevels.kOut));
    

      //m_driverController.y().whileTrue(new RunElevatorCommand(elevatorSystem, m_driverController.getLeftTriggerAxis()));
      m_driverController.back().whileTrue(new ZeroElevatorCommand(elevatorSystem));

  //   }
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Intake score", new RunIntakeCommand(intake, Constants.IntakeConstants.IntakePowerLevels.kIn, vision).withTimeout(0.5));
    NamedCommands.registerCommand("Intake in", new RunIntakeCommand(intake, Constants.IntakeConstants.IntakePowerLevels.kIn, vision).withTimeout(3));
    NamedCommands.registerCommand("Elevator L1", Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel1)));
    NamedCommands.registerCommand("Elevator L2", Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel2)));
    NamedCommands.registerCommand("Elevator L3", Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel3)));
    NamedCommands.registerCommand("Elevator L4", Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel4)));
    NamedCommands.registerCommand("lvl4 pt2", Commands.runOnce(() -> elevatorSystem.setWristTarget(67)));
    NamedCommands.registerCommand("drive to score pose I", driveBase.driveToPose(new Pose2d(new Translation2d(4.937, 4.871), Rotation2d.fromDegrees(-116))));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    // Command AutoCommand = new SequentialCommandGroup(driveBase.getAutonomousCommand("Red Side Test"), new WaitCommand(1), new RunIntakeCommand(intake, 0, vision));

    return autoChooser.getSelected();


  }
}
