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
import swervelib.SwerveDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

public class VisionSubsystem extends SubsystemBase {

  //  private static DigitalInput DIO_1;
  //  private boolean output1;
  //  private boolean[] sensorValues = new boolean[2];
  private LaserCan lidar;
  private LaserCan.Measurement measurement;
  private SwerveSubsystem swerve;

  private InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap shooterAngleMap = new InterpolatingDoubleTreeMap();

  ChassisSpeeds currentRobotSpeeds;
  ChassisSpeeds currentFieldSpeeds;
  double speed;
  double direction;
  double omegaSpeed;

  double xSpeed;
  double ySpeed;
  double tangentSpeed;
  double normalSpeed;

  boolean red;
  Pose2d pose;
  Translation2d speaker = new Translation2d(-8.27, 1.45);
  double angle;
  
  double xDistance;
  double yDistance;

  double distanceToSpeaker;
  double angleToSpeaker;
  double radiansToSpeaker;

  public double speedMap;
  public double angleMap;
  public double verticalOffset;


  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(SwerveSubsystem swerve) {


    // DIO_1 = new DigitalInput(1);

    lidar = new LaserCan(22);

    this.swerve = swerve;

    shooterSpeedMap.put(Double.valueOf(1.2), Double.valueOf(3300));
    shooterSpeedMap.put(Double.valueOf(1.5), Double.valueOf(3100));
    shooterSpeedMap.put(Double.valueOf(2), Double.valueOf(3000));
    shooterSpeedMap.put(Double.valueOf(2.5), Double.valueOf(3000));
    shooterSpeedMap.put(Double.valueOf(3), Double.valueOf(3000));
    shooterSpeedMap.put(Double.valueOf(3.5), Double.valueOf(3250));
    shooterSpeedMap.put(Double.valueOf(4), Double.valueOf(3600));
    shooterSpeedMap.put(Double.valueOf(4.5), Double.valueOf(4300));
    shooterSpeedMap.put(Double.valueOf(5.3), Double.valueOf(5000));
    shooterSpeedMap.put(Double.valueOf(6), Double.valueOf(5000));

    shooterAngleMap.put(Double.valueOf(1.2), Double.valueOf(1.5));
    shooterAngleMap.put(Double.valueOf(1.5), Double.valueOf(4.5));
    shooterAngleMap.put(Double.valueOf(2), Double.valueOf(11));
    shooterAngleMap.put(Double.valueOf(2.5), Double.valueOf(15.5));
    shooterAngleMap.put(Double.valueOf(3), Double.valueOf(17.5));
    shooterAngleMap.put(Double.valueOf(3.5), Double.valueOf(22.5));
    shooterAngleMap.put(Double.valueOf(4), Double.valueOf(26));
    shooterAngleMap.put(Double.valueOf(4.5), Double.valueOf(28));
    shooterAngleMap.put(Double.valueOf(5.3), Double.valueOf(28.5));
    shooterAngleMap.put(Double.valueOf(6), Double.valueOf(29));


  }

  public double getCoralInIntake() {
    Measurement it = lidar.getMeasurement();
    if (it != null) {
      return it.distance_mm;
    }
    return -999999999;
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
    
    pose = swerve.getVisionPose();
    angle = pose.getRotation().getDegrees();

    if (DriverStation.isFMSAttached()) {
      if (DriverStation.getAlliance().isPresent()) {
        red = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
      }
    }
    else if (pose.getX() > 8.27) red = true;
    else red = false;

    if (red) {
      speaker = new Translation2d(16.54, 5.55);
      xDistance = Math.abs(speaker.getX() - pose.getX());
      yDistance = speaker.getY() - pose.getY();

      if (angle > 0) angle -= 180;
      else angle += 180;
      // angle *= -1;
    }
    else {
      speaker = new Translation2d(0.00, 5.55);
      xDistance = Math.abs(speaker.getX() - pose.getX());
      yDistance = pose.getY() - speaker.getY();
    }

    distanceToSpeaker = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
    angleToSpeaker = (Math.atan2(yDistance, xDistance) * (180/Math.PI));  // Angular offset from test values - 2
    if (red) angleToSpeaker -= 1;
    else angleToSpeaker += 2;
    radiansToSpeaker = Math.atan2(yDistance, xDistance) + 0.035;

    speedMap = shooterSpeedMap.get(distanceToSpeaker);
    angleMap = shooterAngleMap.get(Double.valueOf(distanceToSpeaker));


    currentRobotSpeeds = swerve.getRobotRelativeSpeeds();
    currentFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(currentRobotSpeeds, new Rotation2d(angle));
    speed = currentRobotSpeeds.vxMetersPerSecond;
    direction = currentRobotSpeeds.vyMetersPerSecond; // Direction
    omegaSpeed = currentRobotSpeeds.omegaRadiansPerSecond;

    xSpeed = speed * Math.sin(direction);
    ySpeed = speed * Math.cos(direction);
    tangentSpeed = xSpeed * Math.cos(angleToSpeaker * (Math.PI/180)) + ySpeed * Math.sin(angleToSpeaker * (Math.PI/180));
    normalSpeed = xSpeed * Math.sin(angleToSpeaker * (Math.PI/180)) + ySpeed * Math.cos(angleToSpeaker * (Math.PI/180));

    verticalOffset = normalSpeed * 7 /* * dist? */ ;

    // This method will be called once per scheduler run

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
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

  public double SpeakerRotation() {

    double rotation = (angle - angleToSpeaker) * (0.02);
    if (red) rotation *= -1;

    if (rotation > 0.4) rotation = 0.4;
    if (rotation < -0.4) rotation = -0.4;

    NetworkTableInstance.getDefault().getTable("Centering").getEntry("Rotation").setDouble(rotation);

    return rotation;
  }



  // TODO understand this  ============================================================================

  public double SpeakerRotation(SwerveSubsystem m_DriveSubsystem) {

    double centeringOffset = tangentSpeed * 40;

    double rotationDifference = (angle - (angleToSpeaker + centeringOffset));

    double rotation = 0.0;

    if (rotationDifference > 25) rotation = 0.3;
    else if (rotationDifference < -25) rotation = -0.3;
    else if (rotationDifference > 0) rotation = rotationDifference * 0.0125 + 0.15;
    else if (rotationDifference < 0) rotation = rotationDifference * 0.0125 - 0.15;
    else rotation = .5;

    // 0 - 10 degrees offset



    NetworkTableInstance.getDefault().getTable("Centering").getEntry("Rotation").setDouble(rotation);

    return rotation;
  }






}