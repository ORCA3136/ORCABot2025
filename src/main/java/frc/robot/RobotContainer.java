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
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ReefCentering;
import frc.robot.subsystems.SwerveSubsystem;
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
  private VisionSubsystem vision = new VisionSubsystem();
  private final SwerveSubsystem driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/ORCA2025"), vision); // where to configure the robot or "choose" it
  private IntakeSubsystem intake = new IntakeSubsystem();
  private ElevatorSubsystem elevatorSystem = new ElevatorSubsystem();
  private ClimberSubsystem climber = new ClimberSubsystem();
  private LEDSubsystem ledSubsystem = new LEDSubsystem();
  private ReefCentering reefCentering = new ReefCentering(driveBase, elevatorSystem);
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
    
    SwerveInputStream driveAngularVelocity  = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                            () -> -m_driverController.getLeftY(), 
                                            () -> -m_driverController.getLeftX())
                                            .withControllerRotationAxis(() -> -m_driverController.getRightX())
                                            .deadband(OperatorConstants.DEADBAND)
                                            .scaleTranslation(0.8)
                                            .allianceRelativeControl(true);

    SwerveInputStream driveRegular = driveAngularVelocity.copy().scaleTranslation(1.1);

    SwerveInputStream driveAngularVelocitySlow = driveAngularVelocity.copy().scaleTranslation(Constants.Limits.MEDIUM_SPEED_FACTOR);

  
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                                            .withControllerHeadingAxis(m_driverController::getRightX, 
                                                            m_driverController::getRightY).headingWhile(false);
                                                          //withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightX  <- change this to Y for special mode

    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);
    SwerveInputStream driveRobotOrientedReefSpeed = driveRobotOriented.copy().scaleTranslation(0.2);

    Command driveFieldOrientedDirectAngle = driveBase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = driveBase.driveFieldOriented(driveRegular);
    Command driveFieldOrientedAngularVelocitySlow = driveBase.driveFieldOriented(driveAngularVelocitySlow);
    
    Command driveRobotOrientedAngularVelocitySuperSlow = driveBase.driveFieldOriented(driveRobotOrientedReefSpeed);

    

    SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                                () -> m_driverController.getLeftY(),
                                                () -> m_driverController.getLeftX())
                                                .withControllerRotationAxis(m_driverController::getRightX)
                                                .deadband(OperatorConstants.DEADBAND)
                                                .scaleTranslation(0.8)
                                                .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
                                                                      .withControllerHeadingAxis(() -> Math.sin(
                                                                                                      m_driverController.getRawAxis(
                                                                                                          4) * Math.PI) * (Math.PI * 2),
                                                                                                  () -> Math.cos(
                                                                                                      m_driverController.getRawAxis(
                                                                                                          4) * Math.PI) *
                                                                                                        (Math.PI * 2))
                                                                      .headingWhile(true);

    Command driveFieldOrientedDirectAngleSim = driveBase.driveFieldOriented(driveDirectAngleSim);



    SwerveInputStream driveForwardAuto = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                            () -> 0.15, 
                                            () -> 0)
                                            .allianceRelativeControl(false);

    Command driveForwardAutoCommand = driveBase.driveFieldOriented(driveForwardAuto);

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
    if (RobotBase.isSimulation())
    {
      driveBase.setDefaultCommand(RobotBase.isSimulation() ? 
                                driveFieldOrientedDirectAngle :
                                driveFieldOrientedDirectAngleSim);
    
      m_driverController.start().onTrue(Commands.runOnce(() -> driveBase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
    else
    {

      driveBase.setDefaultCommand(driveFieldOrientedAngularVelocity);
      intake.setDefaultCommand(new DefaultIntakeCommand(intake, vision, climber));



      m_driverController.start().onTrue(Commands.runOnce(driveBase::zeroGyro));
      m_driverController.back().whileTrue(new ZeroElevatorCommand(elevatorSystem));

      m_driverController.leftStick().whileTrue(driveRobotOrientedAngularVelocitySuperSlow);
      m_driverController.rightStick().whileTrue(driveFieldOrientedAngularVelocitySlow);

      m_driverController.y().whileTrue(new RunElevatorCommand(elevatorSystem, Constants.ElevatorConstants.ElevatorPowerLevels.kUp));
      m_driverController.a().whileTrue(new RunElevatorCommand(elevatorSystem, Constants.ElevatorConstants.ElevatorPowerLevels.kDown));
      m_driverController.b().whileTrue(new RunWristCommand(elevatorSystem, Constants.WristConstants.WristPowerLevels.kOut));
      m_driverController.x().whileTrue(new RunWristCommand(elevatorSystem, Constants.WristConstants.WristPowerLevels.kIn));

      m_driverController.povDown().whileTrue(reefCentering.createPathCommand(ReefCentering.Side.Back).until(() -> reefCentering.haveConditionsChanged()).repeatedly());
      m_driverController.povUp().whileTrue(reefCentering.createPathCommand(ReefCentering.Side.Middle).until(() -> reefCentering.haveConditionsChanged()).repeatedly());
      m_driverController.povLeft().whileTrue(reefCentering.createPathCommand(ReefCentering.Side.Left).until(() -> reefCentering.haveConditionsChanged()).repeatedly());
      m_driverController.povRight().whileTrue(reefCentering.createPathCommand(ReefCentering.Side.Right).until(() -> reefCentering.haveConditionsChanged()).repeatedly());

      m_driverController.axisGreaterThan(3, 0.4).whileTrue(new RunIntakeCommand(intake, Constants.IntakeConstants.IntakePowerLevels.kOut, vision,ledSubsystem));
      // Right Trigger - 3 ^^^^
      m_driverController.axisGreaterThan(2, 0.4).whileTrue(new RunIntakeScoreCommand(intake, elevatorSystem, ledSubsystem));
      // Left Trigger - 2 ^^^^

      m_driverController.rightBumper().whileTrue(new DoubleLidarRoutine(intake, Constants.IntakeConstants.IntakePowerLevels.kFeed, vision, ledSubsystem, climber));
      
      m_driverController.leftBumper().whileTrue(new AutoScoreCommand(intake, elevatorSystem, Constants.IntakeConstants.IntakePowerLevels.kOut, vision));


      // m_secondaryController.button(1).whileTrue(Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel4)));
      // m_secondaryController.button(2).whileTrue(Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel3)));
      // m_secondaryController.button(3).whileTrue(Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel2)));
      // m_secondaryController.button(4).whileTrue(Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel1)));
      // m_secondaryController.button(5).whileTrue(Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kTopAlgae)));
      // m_secondaryController.button(6).whileTrue(Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kProcessor)));
      // m_secondaryController.button(9).whileTrue(Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kBottomAlgae)));
      
      m_secondaryController.button(1).whileTrue(Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kLevel4)));
      m_secondaryController.button(2).whileTrue(Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kLevel3)));
      m_secondaryController.button(3).whileTrue(Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kLevel2)));
      m_secondaryController.button(4).whileTrue(Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kFeederStation)));
      m_secondaryController.button(5).whileTrue(Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kTopAlgae)));
      m_secondaryController.button(6).whileTrue(Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kProcessor)));
      m_secondaryController.button(9).whileTrue(Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kBottomAlgae)));
      
      m_secondaryController.button(7).whileTrue(new RunFunnelCommand(climber, Constants.ClimberConstants.kFunnelSpeed));
      m_secondaryController.button(8).whileTrue(new CappnCrunchCommand(climber, Constants.ClimberConstants.kClimberInSpeed).withTimeout(0.05));
      m_secondaryController.button(10).whileTrue(new RunClimberCommand(climber, Constants.ClimberConstants.kClimberInSpeed));
      m_secondaryController.button(11).whileTrue(new RunClimberCommand(climber, Constants.ClimberConstants.kClimberOutSpeed));
      m_secondaryController.button(12).whileTrue(Commands.runOnce(() -> elevatorSystem.updateMode()));
      m_secondaryController.axisGreaterThan(0, -0.2).whileTrue(new RunClimbSequenceCommand(climber, elevatorSystem, false));
    }
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Intake score", new RunIntakeCommand(intake, Constants.IntakeConstants.IntakePowerLevels.kOut, vision,ledSubsystem).withTimeout(0.2));
    NamedCommands.registerCommand("Auto score", new AutoScoreCommand(intake, elevatorSystem, 0, vision));
    NamedCommands.registerCommand("Intake in", new DoubleLidarRoutine(intake, Constants.IntakeConstants.IntakePowerLevels.kFeed, vision, ledSubsystem, climber));
    NamedCommands.registerCommand("Wait for coral", new WaitForCoralCommand(vision));
    NamedCommands.registerCommand("Coral Centering", new DefaultIntakeCommand(intake, vision, climber));
    NamedCommands.registerCommand("Elevator L1", Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel1)));
    NamedCommands.registerCommand("Elevator L2", Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel2)));
    NamedCommands.registerCommand("Elevator L3", Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel3)));
    NamedCommands.registerCommand("Elevator L4", Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel4)));
    NamedCommands.registerCommand("Elevator Target L1", Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kLevel1)));
    NamedCommands.registerCommand("Elevator Target L4", Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kLevel4)));
    NamedCommands.registerCommand("Elevator Bottom Algae", Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kBottomAlgae)));
    NamedCommands.registerCommand("Elevator Processor", Commands.runOnce(() -> elevatorSystem.setSetpointCommand(ElevatorSubsystem.Setpoint.kProcessor)));
    NamedCommands.registerCommand("Hold Algae", Commands.runOnce(() -> elevatorSystem.setElevatorPower(-0.5)).handleInterrupt(() -> elevatorSystem.setElevatorPower(0)));
    NamedCommands.registerCommand("Release Algae", Commands.runOnce(() -> elevatorSystem.setElevatorPower(0.5)).withTimeout(1).handleInterrupt(() -> elevatorSystem.setElevatorPower(0)));
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
