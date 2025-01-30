package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpoints;
import frc.robot.Constants.SimulationRobotConstants;
import frc.robot.Constants.WristConstants.WristSetpoints;
import frc.robot.Configs;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkMax leftElevator = new SparkMax(Constants.SparkConstants.kLeftElevatorCanId, MotorType.kBrushless);
  private SparkMax rightElevator = new SparkMax(Constants.SparkConstants.kRightElevatorCanId, MotorType.kBrushless);
  private SparkMax wristMotor = new SparkMax(Constants.SparkConstants.kWristCanId, MotorType.kBrushless);

  private SparkClosedLoopController elevatorController = leftElevator.getClosedLoopController();
  private SparkClosedLoopController wristController = wristMotor.getClosedLoopController();

  private RelativeEncoder leftElevatorEncoder = leftElevator.getEncoder();
  private RelativeEncoder rightElevatorEncoder = rightElevator.getEncoder();
  private AbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder();

  private boolean manuallyMoving = true;
  private double elevatorTarget = ElevatorSetpoints.kFeederStation;
  private double wristTarget = WristSetpoints.ks1;

  private final DigitalInput elevatorLimitSwitch = new DigitalInput(0);

  private DCMotor elevatorMotorModel = DCMotor.getNEO(2);
  private DCMotor wristMotorModel = DCMotor.getNEO(1);
  private ElevatorSim elevatorSim = new ElevatorSim(elevatorMotorModel,
      SimulationRobotConstants.kElevatorGearing,
      SimulationRobotConstants.kCarriageMass,
      SimulationRobotConstants.kElevatorDrumRadius,
      SimulationRobotConstants.kMinElevatorHeightMeters,
      SimulationRobotConstants.kMaxElevatorHeightMeters,
      true,
      SimulationRobotConstants.kMinElevatorHeightMeters,
      0.0,
      0.0);
  private SingleJointedArmSim wristSim = new SingleJointedArmSim(wristMotorModel,
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

  private SparkMaxSim elevatorMotorSim;
  private SparkMaxSim wristMotorSim;
  private SparkLimitSwitchSim elevatorLimitSwitchSim;
  private double simulatedWristOffset = 0.0;  // Store an offset for simulation


  // Add Mechanism2D visualization
  private final Mechanism2d mech2d = new Mechanism2d(50, 100);
  private final MechanismRoot2d mechRoot = mech2d.getRoot("Elevator Base", 25, 0);
  private final MechanismLigament2d stage0 = mechRoot.append(new MechanismLigament2d("Stage 0", 36, 90));
  private final MechanismLigament2d stage1 = stage0.append(new MechanismLigament2d("Stage 1", 37, 90));
  private final MechanismLigament2d stage2 = stage1.append(new MechanismLigament2d("Inner Carriage", 19, 90));
  private final MechanismLigament2d wristMech = stage2.append(new MechanismLigament2d("Wrist", 5, 180));


  public ElevatorSubsystem() {
    DataLogManager.start();
    leftElevator.configure(Configs.ElevatorConfigs.leftElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightElevator.configure(Configs.ElevatorConfigs.rightElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    wristMotor.configure(Configs.WristConfigs.wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    elevatorMotorSim = new SparkMaxSim(leftElevator, elevatorMotorModel);
    wristMotorSim = new SparkMaxSim(wristMotor, wristMotorModel);
    elevatorLimitSwitchSim = new SparkLimitSwitchSim(leftElevator, false);
    SmartDashboard.putData("Elevator Mechanism", mech2d);

  }

  public void setElevatorPower(double power) {
    leftElevator.set(power);
    rightElevator.set(power);
    manuallyMoving = true;
    if (!manuallyMoving) {
      DataLogManager.log("Manual control activated for Elevator");
  }
  }

  public void setWristPower(double power) {
    wristMotor.set(power);
    if (!manuallyMoving) {
      DataLogManager.log("Manual control activated for Wrist");
    } 
    manuallyMoving = true;
  }

  public void setSetpoint(double elevatorPosition, double wristAngle) {
    if (!manuallyMoving && isSafeToMove() && !atSetpoint()) {
      elevatorTarget = elevatorPosition;
      wristTarget = wristAngle;
      elevatorController.setReference(elevatorTarget, ControlType.kMAXMotionPositionControl);
      wristController.setReference(wristTarget, ControlType.kMAXMotionPositionControl);
      DataLogManager.log("Setpoint updated: Elevator = " + elevatorTarget + ", Wrist = " + wristTarget);
    }
  }

  public boolean isSafeToMove() {
    return !(getWristAngle() < Constants.Limits.kWristSafetyThreshold || getWristAngle() > 350);
  }

  public boolean atSetpoint() {
    return Math.abs(getElevatorPosition() - elevatorTarget) < 0.01 &&
    Math.abs(getWristAngle() - wristTarget) < 2.0 &&
    Math.abs(leftElevator.getEncoder().getVelocity()) < 0.1 &&
    Math.abs(wristEncoder.getVelocity()) < 0.5;
  }

  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
    SmartDashboard.putNumber("Wrist Angle", getWristAngle());
    SmartDashboard.putBoolean("Manually Moving", manuallyMoving);
  }

  public double getElevatorPosition() {
    return (leftElevatorEncoder.getPosition() + rightElevatorEncoder.getPosition()) / 2.0;
  }

  public double getWristAngle() {
    return wristEncoder.getPosition() * 360.0;
  }
  @Override
  public void simulationPeriodic() {

         // Set inputs to simulation models
    elevatorSim.setInput((leftElevator.getAppliedOutput() + rightElevator.getAppliedOutput()) / 2.0 * RobotController.getBatteryVoltage());
    wristSim.setInput(wristMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    
    // Update simulation models
    elevatorSim.update(0.02);
    wristSim.update(0.02);
    
    // Update simulated encoder values
    double simulatedPosition = elevatorSim.getPositionMeters();
    leftElevatorEncoder.setPosition(simulatedPosition);
    rightElevatorEncoder.setPosition(simulatedPosition);
    
    // Simulate the absolute encoder by storing an offset
    simulatedWristOffset = wristSim.getAngleRads() / (2 * Math.PI); // Convert radians to rotations

    // Calculate stage heights
    double totalHeight = elevatorSim.getPositionMeters();
    double stage0Height = Math.min(totalHeight, 0.91);
    double stage1Height = Math.min(Math.max(totalHeight - 0.91, 0), 0.94);
    double stage2Height = Math.min(Math.max(totalHeight - 1.85, 0), 0.48);
    double wristAngle = 180 - Math.toDegrees(wristSim.getAngleRads());

    // Update visualization
    stage0.setLength(stage0Height * SimulationRobotConstants.kPixelsPerMeter);
    stage1.setLength(stage1Height * SimulationRobotConstants.kPixelsPerMeter);
    stage2.setLength(stage2Height * SimulationRobotConstants.kPixelsPerMeter);
    wristMech.setAngle(wristAngle);
    }
    
  

  public double getWristCurrentTarget() {
    return wristTarget;
  }

  public boolean isManuallyMoving() {
    return manuallyMoving;
  }

  public void disableManualControl() {
    if (manuallyMoving) {
      DataLogManager.log("Manual control disabled, resuming automatic movement.");
    }
    manuallyMoving = false;
    setSetpoint(elevatorTarget, wristTarget);

  }

    // Use this method to get the simulated wrist angle in commands
  public double getSimulatedWristPosition() {
    return simulatedWristOffset;
  }
}
