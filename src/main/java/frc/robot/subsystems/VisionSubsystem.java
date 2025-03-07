// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.PoseEstimate;
import swervelib.SwerveDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.AngularVelocity;

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

  boolean hasTarget = false;

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

  /**THIS IS THE METHOD THAT IS BEING CALLED BY PERIODIC IN OUR SWERVE DRIVE */
  public void updatePosesEstimator(SwerveDrive swerve) {
    double maxta = 0.4;
    String camera = null;
    String[] limelights = {"limelight-left", "limelight-right"}; // , "limelight-rear"
    for (String limelight: limelights) {
      if (LimelightHelpers.getTV(limelight) && LimelightHelpers.getTA(limelight) > maxta) {
        maxta = LimelightHelpers.getTA(limelight);
        camera = limelight;
      }
    }
    if (camera != null) {
      PoseEstimate poseEst = getEstimatedGlobalPose(camera);
      swerve.addVisionMeasurement(poseEst.pose, poseEst.timestampSeconds);
      SmartDashboard.putBoolean("limelightTV", true);
      hasTarget = true;
    } else {
      hasTarget = false;
      SmartDashboard.putBoolean("limelightTV", false);
    }
  }


  /**
   * Building this out for hopefully a quick test. to try mega tag 2
   * @param swerve
   */
  public void updatePosesEstimatorMT2(SwerveDrive swerve) {

    double maxta = 0;
    String camera = null;
    PoseEstimate mt2 = new PoseEstimate();
    String[] limelights = {"limelight-left", "limelight-right"}; // , "limelight-rear"
    for (String limelight: limelights) {
      LimelightHelpers.SetRobotOrientation(limelight, swerve.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate megaTag2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);
      
      if(megaTag2Pose.tagCount > 0)
      {
        //we have a tag!
        //if the TA is larger than the other camera
        if(LimelightHelpers.getTA(limelight) > maxta)
        {
          maxta = LimelightHelpers.getTA(limelight);
          mt2  = megaTag2Pose;
          camera = limelight;
        }

      }

    }
    if (camera != null) {
      swerve.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
      SmartDashboard.putBoolean("limelightTV", true);
    }
    else {
      SmartDashboard.putBoolean("limelightTV", false);
    }
  }
    
  public PoseEstimate getEstimatedGlobalPose(String limelight) {
    if (LimelightHelpers.getTV(limelight)) {
      hasTarget = true;
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
        poseEsts[num] = null;
      }
      num++;
    }
    return poseEsts;
     // IDK abt ths
  }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    

    measurement = lidar.getMeasurement();
    if (measurement != null) {
      SmartDashboard.putNumber("Lidar distance", measurement.distance_mm);
      SmartDashboard.putNumber("Lidar status", measurement.status);
      if (Math.abs(measurement.distance_mm) < 150) {
        NetworkTableInstance.getDefault().getTable("Lidar").getEntry("Has Target").setBoolean(true);
      } else {
        NetworkTableInstance.getDefault().getTable("Lidar").getEntry("Has Target").setBoolean(false);
      }

    } else {
      NetworkTableInstance.getDefault().getTable("Lidar").getEntry("Has Target").setBoolean(false);
    }
    

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
}