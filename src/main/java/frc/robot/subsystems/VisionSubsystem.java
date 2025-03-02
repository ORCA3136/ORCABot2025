// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import swervelib.SwerveDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;

import java.util.Optional;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

public class VisionSubsystem extends SubsystemBase {

  //  private static DigitalInput DIO_1;
  //  private boolean output1;
  //  private boolean[] sensorValues = new boolean[2];
  private LaserCan lidar;
  private LaserCan.Measurement measurement;



  double xSpeed;
  double ySpeed;
  double tangentSpeed;
  double normalSpeed;

  boolean red;
  

  LimelightHelpers limelight2;


  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {


    // DIO_1 = new DigitalInput(1);

    lidar = new LaserCan(22);

    if (DriverStation.isFMSAttached()) {
      if (DriverStation.getAlliance().isPresent()) {
        red = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
      }
    }


  }

  public double getCoralInIntake() {
    Measurement it = lidar.getMeasurement();
    if (it != null) {
      return it.distance_mm;
    }
    return -999999999;
  }

  public boolean getTV() {
    return LimelightHelpers.getTV("limelight-left");
  }

  public double getTX() {
    return LimelightHelpers.getTX("limelight-left");
  }

  public void updatePoseEstimator(SwerveDrive swerve) {
    PoseEstimate  poseEst = getEstimatedGlobalPose("limelight-left");
      if (poseEst != null) {
        swerve.addVisionMeasurement(poseEst.pose, poseEst.timestampSeconds);
      }
  }

  // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(VisionSubsystem camera) {
  //   Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
  //   return poseEst;
  // }
    
  public PoseEstimate getEstimatedGlobalPose(String limelight) {
    if (LimelightHelpers.getTV(limelight)) {
      PoseEstimate poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
     
      SmartDashboard.putBoolean("limelightTV", LimelightHelpers.getTV(limelight));
      SmartDashboard.putNumber("limelightX", poseEst.pose.getX());
      SmartDashboard.putNumber("limelightY", poseEst.pose.getY());
      return poseEst;
    }
    SmartDashboard.putBoolean("limelightTV", LimelightHelpers.getTV(limelight));
    SmartDashboard.putNumber("limelightX", new PoseEstimate().pose.getX());
    SmartDashboard.putNumber("limelightY", new PoseEstimate().pose.getY());
    return new PoseEstimate(); // IDK abt ths
  }

  public PoseEstimate[] getEstimatedGlobalPose(String[] limelights) {
    PoseEstimate[] poseEsts = new PoseEstimate[limelights.length];
    int num = 0;
    for (String limelight : limelights) {
      if (LimelightHelpers.getTV(limelight)) {
        PoseEstimate poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
        poseEsts[num] = poseEst;
      }
      else {
        poseEsts[num] = new PoseEstimate();
      }
      num++;
    }
    return poseEsts;
     // IDK abt ths
  }
    
  @Override
  public void periodic() {

    // output1 = DIO_1.get();
    // sensorValues[1] = output1;

    measurement = lidar.getMeasurement();
    if (measurement != null) {
      SmartDashboard.putNumber("Lidar distance", measurement.distance_mm);
      SmartDashboard.putNumber("Lidar status", measurement.status);
    }

      

    // This method will be called once per scheduler run

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-left");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Limelight'X'", getTX());



  }




  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // public boolean getIntakeSensor(int sensorNum) {

  //   if (sensorNum > 0 && sensorNum < sensorValues.length) 
  //     return sensorValues[sensorNum];

  //   return false;
  // }


}