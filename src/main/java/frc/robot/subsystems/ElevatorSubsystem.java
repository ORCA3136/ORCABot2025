// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpoints;

public class ElevatorSubsystem extends SubsystemBase {

  private VisionSubsystem vision;

  /** Instantiates elevator motors */
  SparkMax leftElevator = new SparkMax(Constants.SparkConstants.kLeftElevatorCanId, MotorType.kBrushless);
  SparkMax rightElevator = new SparkMax(Constants.SparkConstants.kRightElevatorCanId, MotorType.kBrushless);

  SparkMax wristMotor = new SparkMax(Constants.SparkConstants.kWristCanId, MotorType.kBrushless);

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
    kBottomAlgae;
  }

  private Setpoint currentLevel = ElevatorSubsystem.Setpoint.kFeederStation;

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkClosedLoopController elevatorClosedLoopController =
      leftElevator.getClosedLoopController();

  private SparkClosedLoopController wristClosedLoopController =
      wristMotor.getClosedLoopController();

  private RelativeEncoder elevatorEncoder = leftElevator.getEncoder(); //need a relative encoder to get number of ticks

  private AbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder();
  // Member variables for subsystem state management
  private boolean elevatorReset = false;
  private double elevatorCurrentTarget = Constants.ElevatorConstants.ElevatorSetpoints.kFeederStation;
  private double wristCurrentTarget = Constants.ElevatorConstants.ElevatorSetpoints.kFeederStation;
  
  private boolean changedLevel = false;
  private Setpoint targetSetpoint = Setpoint.kFeederStation;
  private static double distanceToReef = 10;
  private boolean aboveLevel1 = false;
  private boolean manualMode = false;

  private boolean wristManuallyMoving = true;
  private boolean elevatorManuallyMoving = true;

  private final DigitalInput elevatorLimitSwitch;
  private final DigitalInput algaeLimitSwitch;

  
  public ElevatorSubsystem(VisionSubsystem vision) {

    this.vision = vision;

    zeroElevator();
    Configs.ElevatorConfigs.rightElevatorConfig 
          .follow(leftElevator, true);
    
    leftElevator.configure(Configs.ElevatorConfigs.leftElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightElevator.configure(Configs.ElevatorConfigs.rightElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    wristMotor.configure(Configs.WristConfigs.wristMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    elevatorLimitSwitch = new DigitalInput(0);
    algaeLimitSwitch = new DigitalInput(1);
    
  }
  
  // main elevator/wrist movement
  private void moveToSetpointPID() {
    boolean elBool = false;
    double elTarget = 3;

    boolean wristBool = false;
    double wristTarget = 3;


    if (getWristPosition() > Constants.WristConstants.WristSetpoints.unblock + 2 && getWristPosition() < 350) {
      elBool = true;
    } else if (getWristPosition() < Constants.WristConstants.WristSetpoints.unblock - 2 && getElevatorPosition() > Constants.WallConstants.kElevatorAboveTopBar) {
      if (elevatorCurrentTarget < Constants.WallConstants.kElevatorAboveTopBar) { 
        elTarget = Constants.WallConstants.kElevatorAboveTopBar;
      }
    } else if (getWristPosition() > Constants.WristConstants.WristSetpoints.unblock + 2) {
      if (getElevatorPosition() < Constants.WallConstants.kElevatorBelowTopBar) {
        if (elevatorCurrentTarget > Constants.WallConstants.kElevatorBelowTopBar) { 
          elTarget = Constants.WallConstants.kElevatorBelowTopBar;
        } 
      } 
    } else {
      if (elevatorCurrentTarget > Constants.WallConstants.kElevatorBelowBottomBar) {
        elTarget = Constants.WallConstants.kElevatorBelowBottomBar;
      }
    }

    
    if (getElevatorPosition() < Constants.WallConstants.kElevatorBelowBottomBar) {
      wristBool = true;
    } else {
      if (wristCurrentTarget < Constants.WristConstants.WristSetpoints.unblock - 2) {
        wristTarget = Constants.WristConstants.WristSetpoints.unblock;
      }
    }
    
    if (changedLevel) {
      if (Math.abs(getWristPosition() - Constants.WristConstants.WristSetpoints.unblock) < 5) 
        changedLevel = false;
      wristTarget = getWristOffset(Constants.WristConstants.WristSetpoints.unblock, 3, 1);
      wristBool = false;

      elBool = false;
      elTarget = getElevatorPosition();
    }
    else if (Math.abs(getElevatorPosition() - elevatorCurrentTarget) > 0.5) {
      wristBool = false;
      // wristTarget = getWristPosition();
      wristTarget = Constants.WristConstants.WristSetpoints.unblock;
    }

    if (wristCurrentTarget == Constants.WristConstants.WristSetpoints.kAlgae && isSetpointAlgae(currentLevel)) 
    {
      wristBool = true;
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
    // When we have algae higher wrist limit

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

    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Temp Elevator Target").setNumber(elTarget);
    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Temp Wrist Target").setNumber(wristTarget);

  }

  /*
  private void moveToSetpointBetter() {
    boolean elBool = false;
    double elTarget = 3;

    boolean wristBool = false;
    double wristTarget = 3;

    int currentZone;
    int targetZone;
    boolean differentZones = false;
    
    if (getElevatorPosition() < 23) {
      currentZone = 1;
    } else if (getElevatorPosition() < 39) {
      currentZone = 2;
    } else {
      currentZone = 3;
    }

    if (elevatorCurrentTarget < 23) {
      targetZone = 1;
    } else if (elevatorCurrentTarget < 39) {
      targetZone = 2;
    } else {
      targetZone = 3;
    }

    if (currentZone == 1 && targetZone == 3 || currentZone == 3 && targetZone == 1) {
      differentZones = true;
    }

    if (differentZones) {
      elTarget = 31;
    } else if (currentZone == 1) {

      if (getWristPosition() > Constants.WristConstants.WristSetpoints.unblock && getWristPosition() < 350) {
        elBool = true;
      } else if (getWristPosition() > 25) {
        if (getElevatorPosition() < 20) {
          if (elevatorCurrentTarget > 20) { // can wrist go around???
            elTarget = 20;
          } // is elevator just free here
        } 
      //   else if (40 < getElevatorPosition() && getElevatorPosition() < 65) {
      //     if (elevatorCurrentTarget < 40) {
      //       elTarget = 40;
      //     }
      //     if (elevatorCurrentTarget > 65) {
      //       elTarget = 65;
      //     }
      //   }
        
      // } else if (getWristAngle() > 15 && getElevatorPosition() > 40) {
      //   if (elevatorCurrentTarget < 40) {
      //     elTarget = 40;
      //   }
      //   if (elevatorCurrentTarget > 65) {
      //     elTarget = 65;
      //   }
      } else {
        if (elevatorCurrentTarget > 5) {
          elTarget = 5;
        }
      }
    } else if (currentZone == 3) {
      if (getWristPosition() > Constants.WristConstants.WristSetpoints.unblock && getWristPosition() < 350) {
        elBool = true;
      } else if (25 > getWristPosition() && getWristPosition() < Constants.WristConstants.WristSetpoints.unblock) {
        if (getElevatorPosition() < 65 && getElevatorPosition() > 40) {
          if (getElevatorCurrentTarget() > 65) {
            elTarget = 65;
          } else if (getElevatorCurrentTarget() < 40) {
            elTarget = 40;
          }
        }
      }
    }

    if (differentZones) {
      wristTarget = Constants.WristConstants.WristSetpoints.unblock + 10;
    } else if (currentZone == 1) {
      if (getElevatorPosition() < 5) {
        wristBool = true;
      } else if (getElevatorPosition() < 20) {
        if (wristCurrentTarget < 25) {
          wristTarget = 25;
        }
      // } else if (getElevatorPosition() < 40) {
      //   if (wristCurrentTarget < Constants.WristConstants.WristSetpoints.unblock) {
      //     wristTarget = Constants.WristConstants.WristSetpoints.unblock;
      //   }
      // } else if (getElevatorPosition() < 65) {
      //   if (wristCurrentTarget < 15) {
      //     wristTarget = 15;
      //   }
      } else {
        if (wristCurrentTarget < Constants.WristConstants.WristSetpoints.unblock) {
          wristTarget = Constants.WristConstants.WristSetpoints.unblock;
        }
      }
    } else if (currentZone == 3) {
      if (getElevatorPosition() < 40 || getElevatorPosition() > 65) {
        if (wristCurrentTarget < Constants.WristConstants.WristSetpoints.unblock) {
          wristTarget = Constants.WristConstants.WristSetpoints.unblock;
        }
      } else if (getElevatorPosition() < 65) {
        if (wristCurrentTarget < 15) {
          wristTarget = 15;
        }
      }
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

    if (!isWristManuallyMoving()) {
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
  }
  */

  private double getTargetOffset(double target, double pos, double offset, double tolerance){
    if (MathUtil.isNear(target, pos, tolerance)) {
      return target;
    }
    return target + (offset * Math.signum(target - pos)); 
  }

  private double getWristOffset(double target, double offset, double tolerance){
    return getTargetOffset(target, getWristPosition(), offset, tolerance);
  }

  private void wristMoveToSetpoint() {
    // wristPID(wristCurrentTarget);
    wristClosedLoopController.setReference(
        wristCurrentTarget, ControlType.kPosition);
  }

  private void elevatorMoveToSetpoint() {
    elevatorClosedLoopController.setReference(
      elevatorCurrentTarget, ControlType.kPosition);
  }

  private void wristMoveToSetpoint(double pos) {
    // wristPID(wristCurrentTarget);
    wristClosedLoopController.setReference(
        pos, ControlType.kPosition);
  } 

  private void elevatorMoveToSetpoint(double pos) {
    elevatorClosedLoopController.setReference(
      pos, ControlType.kPosition);
  }

  // public void elevatorMoveAgressivelyToSetpoint(double pos) {
  //   leftElevator.set(Math.signum(getElevatorPosition() - pos)); //TODO?

  // }

  /** Set the elevator motor power in the range of [-1, 1]. */
  public void setElevatorPower(double power) {
    leftElevator.set(power);
    setElevatorManuallyMoving(true);
  }
  
  public void setWristPower(double power) {
    wristMotor.set(power);
    setWristManuallyMoving(true);
  }

  public double getWristCurrentTarget() {
    return wristCurrentTarget;
  }

  public double getElevatorCurrentTarget() {
    return elevatorCurrentTarget;
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

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public void setSetpointCommand(Setpoint setpoint) { 
    //return this.runOnce(
        //() -> {

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
              elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.kFeederStation;
              break;
            case kLevel2:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.kLevel2;
              break;
            case kLevel3:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.kLevel3;
              break;
            case kLevel4:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.kLevel4;
              break;
            case kTop:
              elevatorCurrentTarget = ElevatorSetpoints.kBarge;
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.kAlgae;
            break;
            case kBarge:
              elevatorCurrentTarget = ElevatorSetpoints.kBarge;
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.kBarge;
              break;
            case kBottomAlgae:
              elevatorCurrentTarget = Constants.ElevatorConstants.ElevatorSetpoints.kBottomAlgae;
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.kAlgae;
              break;
            case kTopAlgae:
              elevatorCurrentTarget = Constants.ElevatorConstants.ElevatorSetpoints.kTopAlgae;
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.kAlgae;
              break;
            case kProcessor:
              elevatorCurrentTarget = Constants.ElevatorConstants.ElevatorSetpoints.kProcessor;
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.kProcessor;
              break;
            case kUnblock:
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.unblock;
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

  private void updateElevatorHeight() {
    Setpoint newSetpoint;

    if (manualMode) {
      newSetpoint = targetSetpoint;
    } else {

      if (currentLevel == Setpoint.kBottomAlgae || currentLevel == Setpoint.kTopAlgae) {
        if (distanceToReef < Constants.FieldPoses.reefAlgaeElevatorRange) {
          newSetpoint = targetSetpoint;
        } else {
          if (aboveLevel1) {
            newSetpoint = Setpoint.kFeederStation;
          } else {
            newSetpoint = targetSetpoint;
          }
        }
      } else {
        if (distanceToReef < Constants.FieldPoses.reefElevatorRange) {
          newSetpoint = targetSetpoint;
        } else {
          if (aboveLevel1) {
            newSetpoint = Setpoint.kFeederStation;
          } else {
            newSetpoint = targetSetpoint;
          }
        }
      }
      
      if (DriverStation.isAutonomous()) {
        if (distanceToReef < Constants.FieldPoses.reefAutoElevatorRange) {
          newSetpoint = targetSetpoint;
        } else {
          if (aboveLevel1) {
            newSetpoint = Setpoint.kFeederStation;
          } else {
            newSetpoint = targetSetpoint;
          }
        }
      }

      if (vision.hasCoralInFunnel()) {
        newSetpoint = currentLevel;
      }
    }

    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("newSetpoint").setString("" + newSetpoint);
    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("targetSetpoint").setString("" + targetSetpoint);
    if (currentLevel != newSetpoint) {
      setSetpointCommand(newSetpoint);
    }
  }

  public void updateMode() {
    if (!manualMode) {
      if (distanceToReef > Constants.FieldPoses.reefElevatorRange) {
        targetSetpoint = Setpoint.kFeederStation;
      }
    }
    manualMode = !manualMode;
  }

  public void setMode(boolean bool) {
    manualMode = bool;
  }
  
  public static void updateDistanceToReef(double distance) {
    distanceToReef = distance;
    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Distance To Reef").setNumber(distance);
  }

  public static double getDistanceToReef() {
    return distanceToReef;
  }

  public Setpoint getSetpoint() {
    return currentLevel;
  }

  /*
  private void elevatorPID(double position){
    ourNewestPIDControllerYet.setGoal(position);
    m_elevatorSpeed = ourNewestPIDControllerYet.calculate(getElevatorPosition());
    if (ourNewestPIDControllerYet.atGoal()) {
      m_elevatorSpeed = 0;
    }
    
    if (getElevatorPosition() > Constants.Limits.kElevatorMaxHeight && m_elevatorSpeed < 0) {
      m_elevatorSpeed = 0;
    }

    if (!elevatorLimitSwitch.get() && m_elevatorSpeed > 0) {
        m_elevatorSpeed = 0;
      }
    
    ElevatorMove(m_elevatorSpeed); 
  }

  public void wristPID(double position){ //TODO test this function
    newWristPIDController.setGoal(position);
    m_wristSpeed = newWristPIDController.calculate(getWristPosition());
    // Shuffleboard.putNumber
    if (newWristPIDController.atGoal()) {
      m_wristSpeed = 0;
    }
    
    if (getWristPosition() > Constants.Limits.kWristMaxAngle && m_wristSpeed < 0) {
      m_wristSpeed = 0;
    }

    if (getWristPosition() < Constants.Limits.kWristMinAngle && m_wristSpeed > 0) {
      m_wristSpeed = 0;
      }
    
    WristMove(m_wristSpeed); 
  }
  
  public void ElevatorMove(double d) {
    leftElevator.set(-d);
  }

  public void WristMove(double d) {
    wristMotor.set(-d); // negative is in
  }
  */

  public void setWristTarget(double target) {
    wristCurrentTarget = target;
    setWristManuallyMoving(false);
  }

  public void setWristManuallyMoving(boolean bool) {
    wristManuallyMoving = bool;
  }
  
  public boolean isWristManuallyMoving() {
    return wristManuallyMoving;
  }

  public void setElevatorTarget(double target) {
    elevatorCurrentTarget = target;
    setElevatorManuallyMoving(false);
  }
  
  public void setElevatorManuallyMoving(boolean bool) {
    elevatorManuallyMoving = bool;
  }
  public boolean isElevatorManuallyMoving() {
    return elevatorManuallyMoving;
  }

  public double getElevatorPosition() {
    return elevatorEncoder.getPosition();// might need to be scaled by the gear ratio
  }

  public double getWristPosition() {
    return wristEncoder.getPosition(); // 
  }

   /** Zero the elevator encoder when the limit switch is pressed. */
   private void zeroElevatorOnLimitSwitch() {
    if ( (getElevatorPosition() < 0 || !elevatorReset) && !elevatorLimitSwitch.get()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      // new PrintCommand("RESETTING ELEVATOR");
      zeroElevator();    
      elevatorReset = true;  
    } else if (elevatorLimitSwitch.get()) {
      elevatorReset = false;
    }
  }

  public boolean hasAlgae() {
    return algaeLimitSwitch.get();
  }

  public boolean isElevatorDown() {
    return !elevatorLimitSwitch.get();
  }

  public void zeroElevator() {
    elevatorEncoder.setPosition(0);
  }

  /**
   * Stop the control loop and motor output.
   */
  public void stop()
  {
    leftElevator.set(0.0);
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

  public boolean atScoringPosition() {
    return MathUtil.isNear(wristCurrentTarget, getWristPosition(), 3);
  }

  @Override
  public void periodic() {
    zeroElevatorOnLimitSwitch();

    if (!wristManuallyMoving || !elevatorManuallyMoving) {
      updateElevatorHeight();
      moveToSetpointPID();
    }

    NetworkTableInstance.getDefault().getTable("Wrist").getEntry("Wrist at feeder").setBoolean(Math.abs(getWristPosition() - 4) < 2);
    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Elevator current target").setNumber(elevatorCurrentTarget);
    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Elevator left output").setNumber(leftElevator.getAppliedOutput());
    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Elevator right output").setNumber(rightElevator.getAppliedOutput());

    SmartDashboard.putNumber("Elevator current target", elevatorCurrentTarget);
    SmartDashboard.putNumber("Elevator current position", getElevatorPosition());
    SmartDashboard.putBoolean("Elevator manually moving", wristManuallyMoving);

    SmartDashboard.putNumber("Wrist current target", wristCurrentTarget);
    SmartDashboard.putNumber("Wrist current position", getWristPosition());
    SmartDashboard.putNumber("Wrist current 'angle'", getWristPosition());

    SmartDashboard.putBoolean("Elevator limit switch", !elevatorLimitSwitch.get());
    SmartDashboard.putBoolean("Algaelimit switch", algaeLimitSwitch.get());
    SmartDashboard.putBoolean("Changed Level", changedLevel);

    NetworkTableInstance.getDefault().getTable("Wrist").getEntry("At Scoring Pos").setBoolean(atScoringPosition());

  }

 

  public void simulationPeriodic() {}
}