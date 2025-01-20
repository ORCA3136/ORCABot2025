// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class VisionSubsystem extends SubsystemBase {

   private static DigitalInput DIO_1;
   private boolean output1;
   private boolean[] sensorValues = new boolean[2];

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {

    DIO_1 = new DigitalInput(1);

  }



  

  

  /*SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            getHeading(),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
            }, 
            new Pose2d());

  public void visionPose(Pose2d pose, double timestamp) {
    m_poseEstimator.addVisionMeasurement(pose, timestamp);
  }*/

  @Override
  public void periodic() {

    output1 = DIO_1.get();
    sensorValues[1] = output1;

    /*if (LimelightHelpers.getTV("limelight-april")) {
      if (DriverStation.isAutonomous())
        if (LimelightHelpers.getTA("limelight-april") > 0.25)
          driveBase.visionPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight-april"), Timer.getFPGATimestamp());
      if (LimelightHelpers.getTA("limelight-april") > 0.125)
        driveBase.visionPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight-april"), Timer.getFPGATimestamp());
    }*/

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public boolean getIntakeSensor(int sensorNum) {

    if (sensorNum > 0 && sensorNum < sensorValues.length) 
      return sensorValues[sensorNum];

    return false;
  }

}
