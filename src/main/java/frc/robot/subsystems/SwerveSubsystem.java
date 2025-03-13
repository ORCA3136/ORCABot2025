// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;

import com.google.flatbuffers.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  SwerveDrive swerveDrive;

  

  public SwerveSubsystem(File directory) {

    try {
     swerveDrive = new SwerveParser(directory).createSwerveDrive(4.0,  // Put in constants
                                                                  new Pose2d(new Translation2d(Meter.of(0),
                                                                                               Meter.of(0)),
                                                                             Rotation2d.fromDegrees(0)));
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
  }

  public SwerveDrive getSwerveDrive(){
    return swerveDrive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
