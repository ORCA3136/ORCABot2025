// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.channels.WritableByteChannel;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Configs.WristConfigs;
import frc.robot.Constants.SparkConstants;
import frc.robot.Constants.WallConstants;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.Limits;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;





public class ElevatorSubsystemYams extends SubsystemBase {

  // private VisionSubsystem vision;

  /* The variables to manage the elevator states */
  public enum Setpoint {
    kFeederStation,
    kProcessor,
    kLevel2,
    kLevel3,
    kLevel4,
    kTop,
    kBarge,
    kUnblock,
    kTopAlgae,
    kBottomAlgae
  }
  private Setpoint currentLevel = Setpoint.kFeederStation;

  /* Elevator and wrist motors */
  private final SparkMax leftElevator = new SparkMax(SparkConstants.kLeftElevatorCanId, MotorType.kBrushless);
  private final SparkMax rightElevator = new SparkMax(SparkConstants.kRightElevatorCanId, MotorType.kBrushless);
  private final SparkMax wristMotor = new SparkMax(SparkConstants.kWristCanId, MotorType.kBrushless);

  /* The physical motors for the Sim - does not have a non sim counterpart */
  private final DCMotor DCelevatorMotors = DCMotor.getNEO(2);
  private final DCMotor DCwristMotor = DCMotor.getNEO(1);

  /* The Sim version of the motors */
  private final SparkSim elevatorMotorsSim = new SparkSim(leftElevator, DCelevatorMotors);
  private final SparkSim wristMotorSim = new SparkSim(wristMotor, DCwristMotor);

  /* Elevator and wrist encoders */
  private final RelativeEncoder elevatorEncoder = leftElevator.getEncoder();
  private final AbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder();

  /* Elevator and wrist feedforwards */
  private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(
    ElevatorConstants.PIDConstants.kS, 
    ElevatorConstants.PIDConstants.kG, 
    ElevatorConstants.PIDConstants.kV,
    ElevatorConstants.PIDConstants.kA
  );
  private final ArmFeedforward wristFeedforward = new ArmFeedforward(
    WristConstants.WristPIDConstants.kWristkS, 
    WristConstants.WristPIDConstants.kWristkG, 
    WristConstants.WristPIDConstants.kWristkV,
    WristConstants.WristPIDConstants.kWristkA
  );

  /* Profiled PID controllers */
  private TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(
    ElevatorConstants.PIDConstants.kMaxVelocity / (2.0 * Math.PI * ElevatorConstants.PhysicalConstants.drumRadiusMeters),
    ElevatorConstants.PIDConstants.kMaxAcceleration / (2.0 * Math.PI * ElevatorConstants.PhysicalConstants.drumRadiusMeters)
  );
  private TrapezoidProfile.Constraints wristConstraints = new TrapezoidProfile.Constraints(
    WristConstants.WristPIDConstants.kMaxVelocity  / (2.0 * Math.PI),
    WristConstants.WristPIDConstants.kMaxAcceleration / (2.0 * Math.PI)
  );
  private ProfiledPIDController elevatorProfiledPIDController = new ProfiledPIDController(
    ElevatorConstants.PIDConstants.kP, 
    ElevatorConstants.PIDConstants.kI, 
    ElevatorConstants.PIDConstants.kD, 
    elevatorConstraints
  );
  private ProfiledPIDController wristProfiledPIDController = new ProfiledPIDController(
    WristConstants.WristPIDConstants.kWristKp, 
    WristConstants.WristPIDConstants.kWristKi, 
    WristConstants.WristPIDConstants.kWristKd, 
    wristConstraints
  );

  /* Limit switches */
  private final DigitalInput elevatorLimitSwitch;
  private final DigitalInput algaeLimitSwitch;

  /* Elevator and wrist sims */
  private final ElevatorSim elevatorSim = new ElevatorSim(
    DCelevatorMotors, // Motor type
    ElevatorConstants.PhysicalConstants.elevatorGearing,
    ElevatorConstants.PhysicalConstants.carraigeMassKilograms,
    ElevatorConstants.PhysicalConstants.drumRadiusMeters,
    ElevatorConstants.PhysicalConstants.minHeightMeters,
    ElevatorConstants.PhysicalConstants.maxHeightMeters,
    true, // Simulate gravity
    0 // Starting height (m))
  );
  private final SingleJointedArmSim wristSim = new SingleJointedArmSim(
    DCwristMotor, // Motor type
    WristConstants.PhysicalConstants.wristGearing,
    WristConstants.PhysicalConstants.momentOfInertia, // Arm moment of inertia - Small value since there are no arm parameters
    WristConstants.PhysicalConstants.armLengthMeters, // Arm length (m) - Small value since there are no arm parameters
    Units.degreesToRadians(Constants.Limits.kWristMinAngle),
    Units.degreesToRadians(Constants.Limits.kWristMaxAngle),
    false, // Simulate gravity - Disable gravity for pivot
    Units.degreesToRadians(WristConstants.PhysicalConstants.wristOffset) // Starting position (rad)
  );


  /* Member variables for subsystem state management */
  private boolean elevatorReset = false;
  private double elevatorCurrentTarget = ElevatorConstants.SetpointPositions.kFeederStation;
  private double wristCurrentTarget = ElevatorConstants.SetpointPositions.kFeederStation;
  
  private boolean changedLevel = false;
  private Setpoint targetSetpoint = Setpoint.kFeederStation;
  private static double distanceToReef = 10;
  private boolean aboveLevel1 = false;
  private boolean manualMode = false;

  private boolean wristManuallyMoving = true;
  private boolean elevatorManuallyMoving = true;

  



  public ElevatorSubsystemYams( /* VisionSubsystem vision */ ) {

    // this.vision = vision;

    zeroElevator();
    Configs.ElevatorConfigs.rightElevatorConfig.follow(leftElevator, true);

    wristProfiledPIDController.disableContinuousInput();
    elevatorProfiledPIDController.disableContinuousInput();
    
    leftElevator.configure(Configs.ElevatorConfigs.leftElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightElevator.configure(Configs.ElevatorConfigs.rightElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    wristMotor.configure(Configs.WristConfigs.wristMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    elevatorLimitSwitch = new DigitalInput(0);
    algaeLimitSwitch = new DigitalInput(1);
  }
  


  /**
   * Control loop function that runs at a fixed frequency.
   * This is used for SparkMAX and SparkFlex controllers to implement
   * closed-loop control outside of the main robot loop.
   */
  private void elevatorPIDcontrolLoop(double goal, ControlType type) {
    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("PID Goal").setNumber(goal);
    switch (type) {
      case kPosition:
        NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Control Type").setDefaultString("Position");
        double currentPos = getElevatorPosition();
        double output = elevatorProfiledPIDController.calculate(currentPos, goal);
        NetworkTableInstance.getDefault().getTable("Elevator").getEntry("PID Output").setNumber(output);
        double velocity = elevatorProfiledPIDController.getSetpoint().velocity;
        double feedforwardOutput = elevatorFeedforward.calculate(velocity);
        NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Feedforward Output").setNumber(feedforwardOutput);
        leftElevator.setVoltage(output + feedforwardOutput);
        break;

      case kVelocity:
        NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Control Type").setDefaultString("Velocity");
        double currentVel = elevatorEncoder.getVelocity();
        double velOutput = elevatorProfiledPIDController.calculate(currentVel, goal);
        double accel = elevatorProfiledPIDController.getSetpoint().velocity - currentVel;
        double velFeedforwardOutput = elevatorFeedforward.calculate(goal, accel);
        double velocityVoltage = velOutput + velFeedforwardOutput;
        leftElevator.setVoltage(velocityVoltage);
        break;

      case kDutyCycle:
        NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Control Type").setDefaultString("Duty Cycle");
        leftElevator.set(goal);
        
      default:
        NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Control Type").setDefaultString("Default (Nothing)");
        break;
    }
  }
  private void wristPIDcontrolLoop(double goal, ControlType type) {
    NetworkTableInstance.getDefault().getTable("Wrist").getEntry("PID Goal").setNumber(goal);
    switch (type) {
      case kPosition:
        NetworkTableInstance.getDefault().getTable("Wrist").getEntry("Control Type").setDefaultString("Position");
        double currentPos = getWristPosition();
        double output = wristProfiledPIDController.calculate(currentPos, goal);
        NetworkTableInstance.getDefault().getTable("Wrist").getEntry("PID Output").setNumber(output);
        double velocity = wristProfiledPIDController.getSetpoint().velocity;
        double feedforwardOutput = wristFeedforward.calculate(0, velocity);
        NetworkTableInstance.getDefault().getTable("Wrist").getEntry("Feedforward Output").setNumber(feedforwardOutput);
        wristMotor.setVoltage(output + feedforwardOutput);
        break;

      case kVelocity:
        NetworkTableInstance.getDefault().getTable("Wrist").getEntry("Control Type").setDefaultString("Velocity");
        double currentVel = wristEncoder.getVelocity();
        double velOutput = wristProfiledPIDController.calculate(currentVel, goal);
        double accel = wristProfiledPIDController.getSetpoint().velocity - currentVel;
        double velFeedforwardOutput = wristFeedforward.calculate(0, goal, accel);
        double velocityVoltage = velOutput + velFeedforwardOutput;
        wristMotor.setVoltage(velocityVoltage);
        break;

      case kDutyCycle:
        NetworkTableInstance.getDefault().getTable("Wrist").getEntry("Control Type").setDefaultString("Duty Cycle");
        wristMotor.set(goal);
        
      default:
        NetworkTableInstance.getDefault().getTable("Wrist").getEntry("Control Type").setDefaultString("Default (Nothing)");
        break;
    }
  }

  public void close() {
    leftElevator.close();
    rightElevator.close();
    wristMotor.close();
  }





  // main elevator/wrist movement
  private void moveToSetpointPID() {
    boolean elBool = false;
    double elTarget = 3;

    boolean wristBool = false;
    double wristTarget = 3;


    if (getWristPosition() > WristConstants.WristSetpoints.unblock + 2 && getWristPosition() < 350) {
      elBool = true;
    } else if (getWristPosition() < WristConstants.WristSetpoints.unblock - 2 && getElevatorPosition() > WallConstants.kElevatorAboveTopBar) {
      if (elevatorCurrentTarget < WallConstants.kElevatorAboveTopBar) { 
        elTarget = WallConstants.kElevatorAboveTopBar;
      }
    } else if (getWristPosition() > WristConstants.WristSetpoints.unblock + 2) {
      if (getElevatorPosition() < WallConstants.kElevatorBelowTopBar) {
        if (elevatorCurrentTarget > WallConstants.kElevatorBelowTopBar) { 
          elTarget = WallConstants.kElevatorBelowTopBar;
        } 
      } 
    } else {
      if (elevatorCurrentTarget > WallConstants.kElevatorBelowBottomBar) {
        elTarget = WallConstants.kElevatorBelowBottomBar;
      }
    }

    
    if (getElevatorPosition() < WallConstants.kElevatorBelowBottomBar) {
      wristBool = true;
    } else {
      if (wristCurrentTarget < WristConstants.WristSetpoints.unblock - 2) {
        wristTarget = WristConstants.WristSetpoints.unblock;
      }
    }
    
    if (changedLevel) {
      if (Math.abs(getWristPosition() - WristConstants.WristSetpoints.unblock) < 5) 
        changedLevel = false;
      wristTarget = getWristOffset(WristConstants.WristSetpoints.unblock, 3, 1);
      wristBool = false;

      elBool = false;
      elTarget = getElevatorPosition();
    }
    else if (Math.abs(getElevatorPosition() - elevatorCurrentTarget) > 0.5) {
      wristBool = false;
      // wristTarget = getWristPosition();
      wristTarget = WristConstants.WristSetpoints.unblock;
    }

    if (wristCurrentTarget == WristConstants.WristSetpoints.kAlgae && getWristPosition() > 100) 
    {
      wristBool = true;
      changedLevel = false;
    }


    if (!isElevatorManuallyMoving()) {
      if (elBool) {
        elevatorMoveToSetpoint();
      } else {
        if (elTarget != 3) {
          elevatorMoveToSetpoint(elTarget);
        }
        else {
          elevatorMoveToSetpoint();
        }
      }
    }

    if (!(isWristManuallyMoving())) {
      if (wristBool) {
        wristMoveToSetpoint();
      } else {
        if (wristTarget != 3) {
          wristMoveToSetpoint(wristTarget);
        }
        else {
          wristMoveToSetpoint();
        }
      }
    }

    elevatorMoveToSetpoint();
    wristMoveToSetpoint();

    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Temp Elevator Target").setNumber(elTarget);
    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Temp Wrist Target").setNumber(wristTarget);
  }

  private double getTargetOffset(double target, double pos, double offset, double tolerance){
    if (MathUtil.isNear(target, pos, tolerance)) {
      return target;
    }
    return target + (offset * Math.signum(target - pos)); 
  }

  private double getWristOffset(double target, double offset, double tolerance){
    return getTargetOffset(target, getWristPosition(), offset, tolerance);
  }

  public void elevatorMoveToSetpoint() {
    elevatorPIDcontrolLoop(elevatorCurrentTarget, ControlType.kPosition);
  }

  private void wristMoveToSetpoint() {
    wristPIDcontrolLoop(wristCurrentTarget, ControlType.kPosition);
  }

  public void elevatorMoveToSetpoint(double pos) {
    elevatorPIDcontrolLoop(pos, ControlType.kPosition);
  }

  public void wristMoveToSetpoint(double pos) {
    wristPIDcontrolLoop(pos, ControlType.kPosition);
  }
  
  /** Set the elevator motor power in the range of [-1, 1]. **/
  public void setElevatorPower(double power) {
    leftElevator.set(power);
    setElevatorManuallyMoving(true);
  }
  
  public void setWristPower(double power) {
    wristMotor.set(power);
    setWristManuallyMoving(true);
  }

  public double getElevatorCurrentTarget() {
    return elevatorCurrentTarget;
  }

  public double getWristCurrentTarget() {
    return wristCurrentTarget;
  }

  public void setTargetSetpoint(Setpoint setpoint) {
    setWristManuallyMoving(false);
    setElevatorManuallyMoving(false);
    targetSetpoint = setpoint;

    switch (targetSetpoint) {
      case kFeederStation:
        aboveLevel1 = false;
        break;
      case kLevel2:
        aboveLevel1 = true;
        break;
      case kLevel3:
        aboveLevel1 = true;
        break;
      case kLevel4:
        aboveLevel1 = true;
        break;
      case kTop:
        aboveLevel1 = true;
        break;
      case kBarge:
        aboveLevel1 = true;
        break;
      case kBottomAlgae:
        aboveLevel1 = true;
        break;
      case kTopAlgae:
        aboveLevel1 = true;
        break;
      case kProcessor:
        aboveLevel1 = false;
        break;
      default:
        break;
    }
  }

  private boolean isSetpointAlgae(Setpoint level) {
    if (level == Setpoint.kBarge || level == Setpoint.kTop || level == Setpoint.kBottomAlgae || level == Setpoint.kTopAlgae) {
      return true;
    }
    return false;
  }

  public boolean isCurrentSetpointAlgae() {
    return isSetpointAlgae(currentLevel);
  }

  public boolean hasAlgae() {
    return algaeLimitSwitch.get();
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public void setSetpointCommand(Setpoint setpoint) {
    if (currentLevel != setpoint || currentLevel == null) {
      if ( !(isSetpointAlgae(setpoint) && isSetpointAlgae(currentLevel))) 
      {
        changedLevel = true;
      }
    }
    currentLevel = setpoint;

    setWristManuallyMoving(false);
    setElevatorManuallyMoving(false);
    switch (setpoint) {
      case kFeederStation:
        elevatorCurrentTarget = ElevatorConstants.SetpointPositions.kFeederStation;
        wristCurrentTarget = WristConstants.WristSetpoints.kFeederStation;
        break;
      case kLevel2:
        elevatorCurrentTarget = ElevatorConstants.SetpointPositions.kLevel2;
        wristCurrentTarget = WristConstants.WristSetpoints.kLevel2;
        break;
      case kLevel3:
        elevatorCurrentTarget = ElevatorConstants.SetpointPositions.kLevel3;
        wristCurrentTarget = WristConstants.WristSetpoints.kLevel3;
        break;
      case kLevel4:
        elevatorCurrentTarget = ElevatorConstants.SetpointPositions.kLevel4;
        wristCurrentTarget = WristConstants.WristSetpoints.kLevel4;
        break;
      case kTop:
        elevatorCurrentTarget = ElevatorConstants.SetpointPositions.kBarge;
        wristCurrentTarget = WristConstants.WristSetpoints.kAlgae;
        break;
      case kBarge:
        elevatorCurrentTarget = ElevatorConstants.SetpointPositions.kBarge;
        wristCurrentTarget = WristConstants.WristSetpoints.kBarge;
        break;
      case kBottomAlgae:
        elevatorCurrentTarget = ElevatorConstants.SetpointPositions.kBottomAlgae;
        wristCurrentTarget = WristConstants.WristSetpoints.kAlgae;
        break;
      case kTopAlgae:
        elevatorCurrentTarget = ElevatorConstants.SetpointPositions.kTopAlgae;
        wristCurrentTarget = WristConstants.WristSetpoints.kAlgae;
        break;
      case kProcessor:
        elevatorCurrentTarget = ElevatorConstants.SetpointPositions.kProcessor;
        wristCurrentTarget = WristConstants.WristSetpoints.kProcessor;
        break;
      case kUnblock:
        wristCurrentTarget = WristConstants.WristSetpoints.unblock;
        break;
      default:
        break;
    }
  }

  private void updateElevatorHeight() {
    Setpoint newSetpoint;

    if (manualMode) {
      newSetpoint = targetSetpoint;
    } 
    // else if (vision.hasCoralInFunnel()) {
    //   newSetpoint = currentLevel;
    // } 
    else if (hasAlgae() && isCurrentSetpointAlgae()) {
      if (isSetpointAlgae(targetSetpoint)) {
        newSetpoint = targetSetpoint;
      } else {
        newSetpoint = currentLevel;
      }
    }
    else {
          if (DriverStation.isAutonomous()) {
            if (distanceToReef < FieldPoses.reefAutoElevatorRange) {
              newSetpoint = targetSetpoint;
            } else {
              if (aboveLevel1) {
                newSetpoint = Setpoint.kFeederStation;
              } else {
                newSetpoint = targetSetpoint;
              }
            }
          } else {
            if (currentLevel == Setpoint.kBottomAlgae || currentLevel == Setpoint.kTopAlgae) {
              if (distanceToReef < FieldPoses.reefAlgaeElevatorRange) {
                newSetpoint = targetSetpoint;
              } else {
                if (aboveLevel1) {
                  newSetpoint = Setpoint.kFeederStation;
                } else {
                  newSetpoint = targetSetpoint;
                }
              }
            } else {
              if (distanceToReef < FieldPoses.reefElevatorRange) {
                newSetpoint = targetSetpoint;
              } else {
                if (aboveLevel1) {
                  newSetpoint = Setpoint.kFeederStation;
                } else {
                  newSetpoint = targetSetpoint;
                }
              }
            }
          }
        }

        newSetpoint = targetSetpoint;

        NetworkTableInstance.getDefault().getTable("Elevator").getEntry("newSetpoint").setString("" + newSetpoint);
        NetworkTableInstance.getDefault().getTable("Elevator").getEntry("targetSetpoint").setString("" + targetSetpoint);
    
        if (currentLevel != newSetpoint) {
          setSetpointCommand(newSetpoint);
        }
    }
  
  public static void updateDistanceToReef(double distance) {
    distanceToReef = distance;
  }

  public static double getDistanceToReef() {
    return distanceToReef;
  }

  public Setpoint getSetpoint() {
    return currentLevel;
  }

  public void setElevatorManuallyMoving(boolean bool) {
    elevatorManuallyMoving = bool;
  }

  public void setWristManuallyMoving(boolean bool) {
    wristManuallyMoving = bool;
  }
  
  public void setElevatorTarget(double target) {
    elevatorCurrentTarget = target;
    setElevatorManuallyMoving(false);
  }

  public void setWristTarget(double target) {
    wristCurrentTarget = target;
    setWristManuallyMoving(false);
  }
  
  public boolean isElevatorManuallyMoving() {
    return elevatorManuallyMoving;
  }

  public boolean isWristManuallyMoving() {
    return wristManuallyMoving;
  }





   /** Zero the elevator encoder when the limit switch is pressed. */
   private void zeroElevatorOnLimitSwitch() {
    if ( (getElevatorPosition() < 0 || !elevatorReset) && !elevatorLimitSwitch.get()) {
      zeroElevator();    
      elevatorReset = true;  
    } else if (elevatorLimitSwitch.get()) {
      elevatorReset = false;
    }
  }

  public void zeroElevator() {
    elevatorEncoder.setPosition(0);
  }

    /**
   * A trigger for when the height is at an acceptable tolerance.
   *
   * @param height    Height in encoder units
   * @param tolerance Tolerance in units.
   * @return {@link Trigger}
   */
  public Trigger atHeight(double height, double tolerance)
  {
    return new Trigger(() -> MathUtil.isNear(height,
                                             elevatorEncoder.getPosition(),
                                             tolerance));
  }

  public boolean atHeight() {
    return MathUtil.isNear(elevatorCurrentTarget, getElevatorPosition(), 0.5);
  }

  /**
   * Get whether the elevator is within tolerance
   * @return If elevator tolerance is within 3
   */
  @Logged(name = "At Scoring Pos")
  public boolean atScoringPosition() {
    return MathUtil.isNear(wristCurrentTarget, getWristPosition(), 3);
  }

  @Override
  public void periodic() {
    
    zeroElevatorOnLimitSwitch();

    if (!wristManuallyMoving && !elevatorManuallyMoving) {
      // Updates wrist and elevator setpoints
      updateElevatorHeight();
    }
    // Controls PID and sets limits
    moveToSetpointPID();

    SmartDashboard.putNumber("Elevator current target", elevatorCurrentTarget);
    SmartDashboard.putBoolean("Elevator manually moving", elevatorManuallyMoving);
    SmartDashboard.putBoolean("Wrist manually moving", wristManuallyMoving);
    SmartDashboard.putNumber("Elevator current position", getElevatorPosition());

    SmartDashboard.putNumber("Wrist current target", wristCurrentTarget);
    SmartDashboard.putNumber("Wrist current position", getWristPosition());

    SmartDashboard.putBoolean("Elevator limit switch", !elevatorLimitSwitch.get());
    SmartDashboard.putBoolean("Algaelimit switch", algaeLimitSwitch.get());
    SmartDashboard.putBoolean("Changed Level", changedLevel);

    NetworkTableInstance.getDefault().getTable("Wrist").getEntry("At Scoring Pos").setBoolean(atScoringPosition());
  }

  

  public void simulationPeriodic() {
    // Meters to Rotations Ratio
    double positionToRotations = (1 / (2.0 * Math.PI * ElevatorConstants.PhysicalConstants.drumRadiusMeters)) * ElevatorConstants.PhysicalConstants.elevatorGearing;

    // Set input voltage from motor controller to simulation
    // Note: This may need to be talonfx.getSimState().getMotorVoltage() as the input
    //elevatorSim.setInput(dcMotor.getVoltage(dcMotor.getTorque(elevatorSim.getCurrentDrawAmps()), elevatorSim.getVelocityMetersPerSecond() * positionToRotations * 2 * Math.PI));
    elevatorSim.setInput(getElevatorVoltage());
    wristSim.setInput(getWristVoltage());

    // Update simulation by 20ms
    elevatorSim.update(0.020);
    wristSim.update(0.020);

    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        wristSim.getCurrentDrawAmps() + elevatorSim.getCurrentDrawAmps()));

    // Convert meters to motor rotations
    double elevatorMotorPosition = elevatorSim.getPositionMeters() * positionToRotations;
    double elevatorMotorVelocity = elevatorSim.getVelocityMetersPerSecond() * positionToRotations;
    double wristMotorPosition = Radians.of(wristSim.getAngleRads() * WristConstants.PhysicalConstants.wristGearing).in(Rotations);
    double wristMotorVelocity = RadiansPerSecond.of(wristSim.getVelocityRadPerSec() * WristConstants.PhysicalConstants.wristGearing).in(RotationsPerSecond);

    elevatorMotorsSim.iterate(elevatorMotorVelocity * 60, RoboRioSim.getVInVoltage(), 0.02);
    wristMotorSim.iterate(wristMotorVelocity * 60, RoboRioSim.getVInVoltage(), 0.02);
  }

  /* Get functions that are logged */
  public double getElevatorPosition() {
    return elevatorEncoder.getPosition();
  }

  public double getElevatorPositionMeters() {
    return elevatorEncoder.getPosition() / ElevatorConstants.PhysicalConstants.elevatorGearing;
  }

  public double getWristPosition() {
    return wristEncoder.getPosition(); 
  }

  public double getWristPositionDegrees() {
    return (getWristPosition() / WristConstants.PhysicalConstants.wristGearing) * 360;
  }

  public double getWristPositionRadians() {
    return (getWristPositionDegrees() * Math.PI) / 180;
  }

  public double getElevatorVoltage() {
    return leftElevator.getAppliedOutput() * leftElevator.getBusVoltage() +
          rightElevator.getAppliedOutput() * rightElevator.getBusVoltage();
  }

  public double getWristVoltage() {
    return wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
  }

  public double getElevatorCurrent() {
    return leftElevator.getOutputCurrent() + rightElevator.getOutputCurrent();
  }

  public double getWristCurrent() {
    return wristMotor.getOutputCurrent();
  }

  /* Get functions that aren't logged */
  public ElevatorSim getElevatorSimulation() {
    return elevatorSim;
  }

  public SingleJointedArmSim getWristSimulation() {
    return wristSim;
  }

  public double getSimCurrent() {
    return elevatorSim.getCurrentDrawAmps() + wristSim.getCurrentDrawAmps();
  }











}