// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RunElevatorCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunWristCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/ORCA2025")); // where to configure the robot or "choose" it
  // private SwerveDrive drive;
  private IntakeSubsystem intake = new IntakeSubsystem();
  private WristSubsystem wrist = new WristSubsystem();
  private ElevatorSubsystem elevator = new ElevatorSubsystem();

  BooleanLogEntry myBooleanLog;
  DoubleLogEntry myDoubleLog;
  StringLogEntry myStringLog;
  


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    wrist.setElevator(elevator);
    elevator.setWrist(wrist);   // mildly jank
    wrist.setElevator(elevator);

    // Configure the trigger bindings
    configureBindings();

    DataLogManager.start();
    // DataLog log = DataLogManager.getLog();
    // myBooleanLog = new BooleanLogEntry(log, "/my/boolean");
    // myDoubleLog = new DoubleLogEntry(log, "/my/double");
    // myStringLog = new StringLogEntry(log, "/my/string");

  }

  SwerveInputStream driveAngularVelocity  = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                            () -> -m_driverController.getLeftY(), 
                                            () -> -m_driverController.getLeftX())
                                            .withControllerRotationAxis(m_driverController::getRightX)
                                            .deadband(OperatorConstants.DEADBAND)
                                            .scaleTranslation(0.8)
                                            .allianceRelativeControl(true);

  
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                                          .withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightY).headingWhile(false);
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
    driveBase.setDefaultCommand(driveFieldOrientedDirectAngle);

    // if (RobotBase.isSimulation())
    // {
    //   driveBase.setDefaultCommand(RobotBase.isSimulation() ? 
    //                             driveFieldOrientedDirectAngle :
    //                             driveFieldOrientedDirectAngleSim);
    // }
    if (Robot.isSimulation())
    {
      m_driverController.start().onTrue(Commands.runOnce(() -> driveBase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
    else
    {
      m_driverController.start().onTrue(Commands.runOnce(driveBase::zeroGyro));


      // m_driverController.leftBumper().whileTrue(new RunElevatorCommand(elevator, wrist, Constants.ElevatorConstants.ElevatorPowerLevels.kUp));
      // m_driverController.rightBumper().whileTrue(new RunElevatorCommand(elevator, wrist, Constants.ElevatorConstants.ElevatorPowerLevels.kDown));

      //m_driverController.leftBumper().onTrue(Commands.runOnce(  () -> {elevator.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel4);       wrist.setSetpointCommand(WristSubsystem.Setpoint.ks3);}));
      //m_driverController.rightBumper().onTrue(Commands.runOnce( () -> {elevator.setSetpointCommand(ElevatorSubsystem.Setpoint.kFeederStation);wrist.setSetpointCommand(WristSubsystem.Setpoint.ks1);}));

      
      // m_driverController.a().onTrue(Commands.runOnce( () -> elevator.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel2)));
      // m_driverController.b().onTrue(Commands.runOnce( () -> elevator.setSetpointCommand(ElevatorSubsystem.Setpoint.kLevel3)));
      // m_driverController.x().onTrue(Commands.runOnce( () -> wrist.setSetpointCommand(WristSubsystem.Setpoint.ks1)));
      // m_driverController.y().onTrue(new RunIntakeCommand(intake, Constants.IntakeConstants.IntakePowerLevels.kIn));

      m_driverController.a().whileTrue(new RunElevatorCommand(elevator, wrist, Constants.ElevatorConstants.ElevatorPowerLevels.kDown));
      m_driverController.y().whileTrue(new RunElevatorCommand(elevator, wrist, Constants.ElevatorConstants.ElevatorPowerLevels.kUp));
      m_driverController.x().whileTrue(new RunWristCommand(wrist, Constants.WristConstants.WristPowerLevels.kUp));
      m_driverController.b().whileTrue(new RunWristCommand(wrist, Constants.WristConstants.WristPowerLevels.kDown));


      // m_driverController.axisGreaterThan(3, 0.5).whileTrue(new RunWristCommand(wrist, Constants.WristConstants.WristPowerLevels.kDown));  // was used in testing
      // m_driverController.axisGreaterThan(2, 0.5).whileTrue(new RunWristCommand(wrist, Constants.WristConstants.WristPowerLevels.kUp));

      m_driverController.axisGreaterThan(3, 0.4).whileTrue(new RunIntakeCommand(intake, Constants.IntakeConstants.IntakePowerLevels.kIn));
      m_driverController.axisGreaterThan(2, 0.4).whileTrue(new RunIntakeCommand(intake, Constants.IntakeConstants.IntakePowerLevels.kOut));

    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null; //Autos.exampleAuto(m_exampleSubsystem);
  }
}
