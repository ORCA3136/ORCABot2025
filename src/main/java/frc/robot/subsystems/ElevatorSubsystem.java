// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpoints;
import frc.robot.Constants.SimulationRobotConstants;
import frc.robot.Configs;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorSubsystem extends SubsystemBase {

  /** Instantiates elevator motors */
  SparkMax leftElevator = new SparkMax(Constants.SparkConstants.kLeftElevatorCanId, MotorType.kBrushless);
  SparkMax rightElevator = new SparkMax(Constants.SparkConstants.kRightElevatorCanId, MotorType.kBrushless);

  SparkMax wristMotor = new SparkMax(Constants.SparkConstants.kWristCanId, MotorType.kBrushless);


  public enum Setpoint {
    kFeederStation,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4,
    kUnblock;
  }

  private boolean wristBlocking;
  private boolean elevatorBlocking;

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkClosedLoopController elevatorClosedLoopController =
      leftElevator.getClosedLoopController();

  private SparkClosedLoopController wristClosedLoopController =
      wristMotor.getClosedLoopController();

  private RelativeEncoder elevatorEncoder = rightElevator.getEncoder(); //need a relative encoder to get number of ticks
   //private RelativeEncoder elevatorEncoder = leftElevator.getEncoder(); we might want to get both left and right encoder and average the values


  private AbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder();
  //wristEncoder.scaledInputs();



  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private double elevatorCurrentTarget = Constants.ElevatorConstants.ElevatorSetpoints.kFeederStation;
  private double wristCurrentTarget = Constants.ElevatorConstants.ElevatorSetpoints.kFeederStation;
  

  private boolean manuallyMoving = true;


  DoubleLogEntry elevatorLog;
  DoubleLogEntry wristLog;

  private boolean wasResetByLimit = false;

  private final DigitalInput elevatorLimitSwitch;

  //some stuff for simulation
  // Simulation setup and variables

  //TODO - WE NEED TO ADJUST THIS TO SUPPORT 2 MOTORS

  private DCMotor elevatorMotorModel = DCMotor.getNEO(2);
  private SparkMaxSim elevatorMotorSim;
  private SparkLimitSwitchSim elevatorLimitSwitchSim;
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          elevatorMotorModel,
          SimulationRobotConstants.kElevatorGearing,
          SimulationRobotConstants.kCarriageMass,
          SimulationRobotConstants.kElevatorDrumRadius,
          SimulationRobotConstants.kMinElevatorHeightMeters,
          SimulationRobotConstants.kMaxElevatorHeightMeters,
          true,
          SimulationRobotConstants.kMinElevatorHeightMeters,
          0.0,
          0.0);

  
  private DCMotor armMotorModel = DCMotor.getNEO(1);
  private SparkMaxSim armMotorSim;
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          armMotorModel,
          SimulationRobotConstants.kArmReduction,
          SingleJointedArmSim.estimateMOI(
              SimulationRobotConstants.kArmLength, SimulationRobotConstants.kArmMass),
          SimulationRobotConstants.kArmLength,
          SimulationRobotConstants.kMinAngleRads,
          SimulationRobotConstants.kMaxAngleRads,
          true,
          SimulationRobotConstants.kMinAngleRads,
          0.0,
          0.0);

            // Mechanism2d setup for subsystem
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("ElevatorArm Root", 25, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Elevator",
              SimulationRobotConstants.kMinElevatorHeightMeters
                  * SimulationRobotConstants.kPixelsPerMeter,
              90));
  private final MechanismLigament2d m_armMech2d =
      m_elevatorMech2d.append(
          new MechanismLigament2d(
              "Arm",
              SimulationRobotConstants.kArmLength * SimulationRobotConstants.kPixelsPerMeter,
              180 - Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads) - 90));

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {
    // Zero elevator encoders on initialization
    // leftElevatorEncoder.setPosition(0);
    // rightElevatorEncoder.setPosition(0);
    // <l/r>ElevatorEncoder.setPosition(0);

  // Starts recording to data log
  DataLogManager.start();
  // Set up custom log entries
  DataLog log = DataLogManager.getLog();

  wristLog = new DoubleLogEntry(log, "/wrist/angle");
  elevatorLog = new DoubleLogEntry(log, "/elevator/position");
    
    Configs.ElevatorConfigs.rightElevatorConfig
          .follow(leftElevator, false);
    
    leftElevator.configure(Configs.ElevatorConfigs.leftElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightElevator.configure(Configs.ElevatorConfigs.rightElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wristMotor.configure(Configs.WristConfigs.wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // Display mechanism2d
    SmartDashboard.putData("Elevator Subsystem", m_mech2d);

    // Initialize simulation values
    elevatorMotorSim = new SparkMaxSim(leftElevator, elevatorMotorModel);
    elevatorLimitSwitchSim = new SparkLimitSwitchSim(leftElevator, false);
    armMotorSim = new SparkMaxSim(wristMotor, armMotorModel);

    elevatorLimitSwitch = new DigitalInput(0);

  }

  private void moveToSetpoint() {
    elevatorClosedLoopController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
    wristClosedLoopController.setReference(
        wristCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  private void wristMoveToSetpoint() {
    wristClosedLoopController.setReference(
        wristCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  private void elevatorMoveToSetpoint() {
    elevatorClosedLoopController.setReference(
      elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  /** Set the elevator motor power in the range of [-1, 1]. */
  public void setElevatorPower(double power) {
    leftElevator.set(power);
    setManuallyMoving(true);
  }
  
  public void setWristPower(double power) {
    wristMotor.set(power);
    setManuallyMoving(true);
  }

  public double getWristCurrentTarget() {
    return wristCurrentTarget;
  }

  public double getElevatorCurrentTarget() {
    return elevatorCurrentTarget;
  }

  // private Angle convertSensorUnitsToAngle(Angle measurement) {
  //   return Rotations.of(measurement.in(Rotations)/268); // make a constant -> WristReduction (AKA grar ratio)
  // }

  public double getWristAngle() {
    return 360 - getWristPosition() * 360;
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public void setSetpointCommand(Setpoint setpoint) {  // see wrist subsystem counterpart
    DataLogManager.log("setpoint command");
    //return this.runOnce(
        //() -> {
          setManuallyMoving(false);
          switch (setpoint) {
            case kFeederStation:
              elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.ks1;
              break;
            case kLevel1:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.kLevel1;
              break;
            case kLevel2:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.kLevel1;
              break;
            case kLevel3:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.kLevel1;
              break;
            case kLevel4:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.kLevel4;
              break;
            case kUnblock:
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.unblock;
              break;
          }
  }
  

  public void setManuallyMoving(boolean bool) {
    manuallyMoving = bool;
  }
  
  public boolean isManuallyMoving() {
    return manuallyMoving;
  }
  

  public double getElevatorPosition() {
    return elevatorEncoder.getPosition();// might need to be scaled by the gear ratio
  }

  public double getWristPosition() {
    return wristEncoder.getPosition(); // -1 because of the direction of rotation
  }

  private void setWristBlocking(boolean bool) {
    wristBlocking = bool;
  }

  private void setElevatorBlocking(boolean bool) {
    elevatorBlocking = bool;
  }

  public boolean elevatorInTheWay() {  // ported from separate subsystem times, need to redefine
    return elevatorBlocking;
  }

  public boolean wristInTheWay() {
    return wristBlocking;
  }

   /** Zero the elevator encoder when the limit switch is pressed. */
   private void zeroElevatorOnLimitSwitch() {
    if (!wasResetByLimit && leftElevator.getReverseLimitSwitch().isPressed()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      elevatorEncoder.setPosition(0);
      wasResetByLimit = true;
    } else if (!leftElevator.getReverseLimitSwitch().isPressed()) {
      wasResetByLimit = false;
    }
  }

  /*public boolean elevatorCanMove() {
    if (wrist.isWristInTheWay()) {
      return false;
    }
    return true;
  }

  public void makeElevatorMovable() {
    if (!elevatorCanMove()) {
      wrist.setSetpointCommand(WristSubsystem.Setpoint.unblock);
    }
  }  */


  @Override
  public void periodic() {
    zeroElevatorOnLimitSwitch();
    //if () {

    //}
    if (!manuallyMoving) {
      // if (!wrist.isWristInTheWay()) {

        moveToSetpoint();
      // } else {
      //   wrist.setSetpointCommand(WristSubsystem.Setpoint.unblock);
      // }
      
      
    }

    if (getWristAngle() < Constants.Limits.kWristSafetyThreshold || getWristAngle() > 350) {
      //DataLogManager.log("Robot thinks the elevator is in the way");  // silly robit
      setWristBlocking(true);
    } else {
      setWristBlocking(false);
    }

    SmartDashboard.putNumber("Elevator current target", elevatorCurrentTarget);
    SmartDashboard.putNumber("Elevator current position", getElevatorPosition());
    SmartDashboard.putBoolean("Elevator manually moving", manuallyMoving);
    SmartDashboard.putBoolean("Elevator blocking status", elevatorInTheWay());

    SmartDashboard.putNumber("Wrist current target", wristCurrentTarget);
    SmartDashboard.putNumber("Wrist current position", getWristPosition());
    SmartDashboard.putBoolean("Wrist blocking status", wristInTheWay());
    SmartDashboard.putNumber("Wrist current 'angle'", getWristAngle());
    SmartDashboard.putBoolean("limit switch", elevatorLimitSwitch.get());

    elevatorLog.append(getElevatorPosition());
    wristLog.append(getWristAngle());

     // Update mechanism2d
    m_elevatorMech2d.setLength(
      SimulationRobotConstants.kPixelsPerMeter * SimulationRobotConstants.kMinElevatorHeightMeters
          + SimulationRobotConstants.kPixelsPerMeter
              * (elevatorEncoder.getPosition() / SimulationRobotConstants.kElevatorGearing)
              * (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI));

    m_armMech2d.setAngle(
        180
            - ( // mirror the angles so they display in the correct direction
            Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads)
                + Units.rotationsToDegrees(
                    wristEncoder.getPosition() / SimulationRobotConstants.kArmReduction))
            - 90 // subtract 90 degrees to account for the elevator
        );
    
    // This method will be called once per scheduler run
  }

   /** Get the current drawn by each simulation physics model */
   public double getSimulationCurrentDraw() {
    return m_elevatorSim.getCurrentDrawAmps();
  }

 public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(leftElevator.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_armSim.setInput(wristMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Update sim limit switch
    elevatorLimitSwitchSim.setPressed(m_elevatorSim.getPositionMeters() == 0);

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);
    m_armSim.update(0.020);

    // Iterate the elevator and arm SPARK simulations
    elevatorMotorSim.iterate(
        ((m_elevatorSim.getVelocityMetersPerSecond()
                    / (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI))
                * SimulationRobotConstants.kElevatorGearing)
            * 60.0,
        RobotController.getBatteryVoltage(),
        0.02);
    armMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(
            m_armSim.getVelocityRadPerSec() * SimulationRobotConstants.kArmReduction),
        RobotController.getBatteryVoltage(),
        0.02);
    
    // SimBattery is updated in Robot.java
  }
}