// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.Drake;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.drive.motors.DriveMotor;
import frc.robot.subsystems.vision.Vision;

public class TrackAprilTags extends CommandBase {
  /** Creates a new TrackAprilTags. */
  private Vision vision;
  @SuppressWarnings("all")
  private SwerveSubsystem swerveSubsystem;
  public TrackAprilTags(Vision vision, SwerveSubsystem swerveSubsystem) {
    this.vision=vision;
    this.swerveSubsystem=swerveSubsystem;
    addRequirements(vision);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isAprilTagInView = vision.isConnected()&&vision.hasTargets();//replace with method from vision that checks if there are apriltags in view
    if(isAprilTagInView){
      //follow april tag
      Pair<Pose2d, Double> visionResult = vision.getEstimatedGlobalPose(swerveSubsystem.getPose());
      double aprilTagAngleInFrame=visionResult.getFirst().getRotation().getDegrees();//replace with method from vision that returns the angle of the tag within view of the camera
      vision.setServoAngle((vision.getServoAngle()-aprilTagAngleInFrame)%180);
      swerveSubsystem.updateVisionMeasurement(visionResult.getFirst(), visionResult.getSecond());
    }else{
      wander();
    }
  }
  public void wander(){
    if(Math.abs(vision.getLastCommandedServoAngle()-vision.getServoAngle()) < 1e-1){
      double nextCommmandedAngle=MathUtil.applyDeadband(vision.getLastCommandedServoAngle(),1e-1)==180?0:180;
      vision.setServoAngle(nextCommmandedAngle);
      SmartDashboard.putNumber("servoAngle", vision.getServoAngle());
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
