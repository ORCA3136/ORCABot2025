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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpoints;
import frc.robot.Constants.SimulationRobotConstants;

public class ElevatorSubsystem extends SubsystemBase {

  /** Instantiates elevator motors */
  SparkMax leftElevator = new SparkMax(Constants.SparkConstants.kLeftElevatorCanId, MotorType.kBrushless);
  SparkMax rightElevator = new SparkMax(Constants.SparkConstants.kRightElevatorCanId, MotorType.kBrushless);

  SparkMax wristMotor = new SparkMax(Constants.SparkConstants.kWristCanId, MotorType.kBrushless);

  public enum Setpoint {
    kFeederStation,
    kProcessor,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4,
    kUnblock,
    kTopAlgae,
    kBottomAlgae;
  }

  private Setpoint currentLevel;

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkClosedLoopController elevatorClosedLoopController =
      leftElevator.getClosedLoopController();

  private SparkClosedLoopController wristClosedLoopController =
      wristMotor.getClosedLoopController();

  private RelativeEncoder elevatorEncoder = leftElevator.getEncoder(); //need a relative encoder to get number of ticks
   //private RelativeEncoder elevatorEncoder = leftElevator.getEncoder(); we might want to get both left and right encoder and average the values

  private AbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder();

  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private double elevatorCurrentTarget = Constants.ElevatorConstants.ElevatorSetpoints.kFeederStation;
  private double wristCurrentTarget = Constants.ElevatorConstants.ElevatorSetpoints.kFeederStation;
  
  private boolean changedLevel = false;
  

  private boolean wristManuallyMoving = true;
  private boolean elevatorManuallyMoving = true;

  private final DigitalInput elevatorLimitSwitch;


  // Simulation setup and variables
  //TODO - WE NEED TO ADJUST THIS TO SUPPORT 2 MOTORS

  private DCMotor elevatorMotorModel = DCMotor.getNEO(2);
  private SparkMaxSim elevatorMotorSim;
  // private SparkLimitSwitchSim elevatorLimitSwitchSim;
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

    zeroElevator();
    Configs.ElevatorConfigs.rightElevatorConfig 
         .follow(leftElevator, true);
    
    leftElevator.configure(Configs.ElevatorConfigs.leftElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightElevator.configure(Configs.ElevatorConfigs.rightElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    wristMotor.configure(Configs.WristConfigs.wristMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
    // Display mechanism2d
    SmartDashboard.putData("Elevator Subsystem", m_mech2d);

    // Initialize simulation values
    elevatorMotorSim = new SparkMaxSim(leftElevator, elevatorMotorModel);
    // elevatorLimitSwitchSim = new SparkLimitSwitchSim(leftElevator, false);
    armMotorSim = new SparkMaxSim(wristMotor, armMotorModel);

    elevatorLimitSwitch = new DigitalInput(0);

  }
  
  // main elevator/wrist movement
  private void moveToSetpointPID() {
    boolean elBool = false;
    double elTarget = 3;

    boolean wristBool = false;
    double wristTarget = 3;


    if (getWristAngle() > Constants.WristConstants.WristSetpoints.unblock && getWristAngle() < 350) {
      elBool = true;
    } else if (getWristAngle() < Constants.WristConstants.WristSetpoints.unblock && getElevatorPosition() > 90) {
      if (elevatorCurrentTarget < 90) { 
        elTarget = 90;
      }
    } else if (getWristAngle() > 22) {
      if (getElevatorPosition() < 22) {
        if (elevatorCurrentTarget > 22) { 
          elTarget = 22;
        } 
      } 
    } else {
      if (elevatorCurrentTarget > 5) {
        elTarget = 5;
      }
    }

    
    if (getElevatorPosition() > 90) {
      if (wristCurrentTarget < 50) {
        wristTarget = 50;
      }
    } else if (getElevatorPosition() < 5) {
      wristBool = true;
    } else if (getElevatorPosition() < 20) {
      if (wristCurrentTarget < 22) {
        wristTarget = 22;
      }
    } else {
      if (wristCurrentTarget < Constants.WristConstants.WristSetpoints.unblock) {
        wristTarget = Constants.WristConstants.WristSetpoints.unblock;
      }
    }

    
    if (changedLevel) {
      if (Math.abs(getWristPosition() - 107) < 1) changedLevel = false;
      wristTarget = 107;
      wristBool = false;

      elBool = false;
      elTarget = getElevatorPosition();
    }
    else if (Math.abs(getElevatorPosition() - elevatorCurrentTarget) > 0.5) {
      wristBool = false;
      wristTarget = getWristPosition();
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

    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Temp Elevator Target").setNumber(elTarget);
    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Temp Wrist Target").setNumber(wristTarget);

  }

  // not currently in use 
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

      if (getWristAngle() > Constants.WristConstants.WristSetpoints.unblock && getWristAngle() < 350) {
        elBool = true;
      } else if (getWristAngle() > 25) {
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
      if (getWristAngle() > Constants.WristConstants.WristSetpoints.unblock && getWristAngle() < 350) {
        elBool = true;
      } else if (25 > getWristAngle() && getWristAngle() < Constants.WristConstants.WristSetpoints.unblock) {
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

  private void wristMoveToSetpoint() {
    wristClosedLoopController.setReference(
        wristCurrentTarget, ControlType.kPosition);
  }

  public void elevatorMoveToSetpoint() {
    elevatorClosedLoopController.setReference(
      elevatorCurrentTarget, ControlType.kPosition);
  }

  public void wristMoveToSetpoint(double pos) {
    wristClosedLoopController.setReference(
        pos, ControlType.kPosition);
  }

  public void elevatorMoveToSetpoint(double pos) {
    elevatorClosedLoopController.setReference(
      pos, ControlType.kPosition);
  }

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

  public double getWristAngle() {
    return getWristPosition();
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public void setSetpointCommand(Setpoint setpoint) {  // see wrist subsystem counterpart
    //return this.runOnce(
        //() -> {
          if (currentLevel != setpoint || currentLevel == null) {
            changedLevel = true;
          }
          currentLevel = setpoint;

          setWristManuallyMoving(false);
          setElevatorManuallyMoving(false);
          switch (setpoint) {
            case kFeederStation:
              elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.kFeederStation;
              break;
            case kLevel1:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.kLevel1;
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
            case kBottomAlgae:
              elevatorCurrentTarget = Constants.ElevatorConstants.ElevatorSetpoints.kBottomAlgae;
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.kAlgae1;
              break;
            case kTopAlgae:
              elevatorCurrentTarget = Constants.ElevatorConstants.ElevatorSetpoints.kTopAlgae;
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.kAlgae2;
              break;
            case kProcessor:
              elevatorCurrentTarget = Constants.ElevatorConstants.ElevatorSetpoints.kProcessor;
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.kProcessor;
              break;
            case kUnblock:
              wristCurrentTarget = Constants.WristConstants.WristSetpoints.unblock;
              break;
          }
  }

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
    if (getElevatorPosition() < 0 && !elevatorLimitSwitch.get()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      new PrintCommand("RESETTING ELEVATOR");
      zeroElevator();      
    }
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




  @Override
  public void periodic() {
    zeroElevatorOnLimitSwitch();

    if (!wristManuallyMoving || !elevatorManuallyMoving) {
      moveToSetpointPID();
    }

    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Elevator current target").setNumber(elevatorCurrentTarget);
    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Elevator left output").setNumber(leftElevator.getAppliedOutput());
    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Elevator right output").setNumber(rightElevator.getAppliedOutput());

    SmartDashboard.putNumber("Elevator current target", elevatorCurrentTarget);
    SmartDashboard.putNumber("Elevator current position", getElevatorPosition());
    SmartDashboard.putBoolean("Elevator manually moving", wristManuallyMoving);

    SmartDashboard.putNumber("Wrist current target", wristCurrentTarget);
    SmartDashboard.putNumber("Wrist current position", getWristPosition());
    SmartDashboard.putNumber("Wrist current 'angle'", getWristAngle());

    SmartDashboard.putBoolean("limit switch", !elevatorLimitSwitch.get());
    SmartDashboard.putBoolean("Changed Level", changedLevel);

    

    if (RobotBase.isSimulation())
    {
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
    }
  }

   /** Get the current drawn by each simulation physics model */
  public double getSimulationCurrentDraw() {
    return m_elevatorSim.getCurrentDrawAmps();
  }

  public void simulationPeriodic() {
  if (RobotBase.isSimulation()) 
    {
      // In this method, we update our simulation of what our elevator is doing
      // First, we set our "inputs" (voltages)
      m_elevatorSim.setInput(elevatorMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
      m_armSim.setInput(armMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

      // Update sim limit switch
      // elevatorLimitSwitchSim.setPressed(m_elevatorSim.getPositionMeters() == 0);

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
}