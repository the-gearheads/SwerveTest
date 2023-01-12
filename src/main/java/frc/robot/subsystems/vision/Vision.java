// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class Vision extends SubsystemBase {
  private final Servo servo;
  private PhotonCamera targetCam;
  private RobotPoseEstimator robotPoseEstimator;
  private double servoAngle;
  public Vision() {
    servo = new Servo(0);
    servoAngle=getLastCommandedServoAngle();
    this.targetCam = new PhotonCamera("target");
    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(targetCam, Constants.Vision.robotToCam));
    this.robotPoseEstimator=new RobotPoseEstimator(Constants.Vision.atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
  }
  public void setServoAngle(double angle){servo.setAngle(angle);}
  public double getLastCommandedServoAngle(){return servo.getAngle();}
  public double getServoAngle(){return this.servoAngle;}

  public boolean isConnected(){
    return targetCam.isConnected();
  }
  public boolean hasTargets(){
    return targetCam.getLatestResult().hasTargets();
  }
  public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    double currentTime = Timer.getFPGATimestamp();
    Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
    if (result.isPresent()) {
        return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
    } else {
        return new Pair<Pose2d, Double>(null, 0.0);
    }
}

  @Override
  public void periodic() {
    //Servo Code
    int servoDirection=getLastCommandedServoAngle()>servoAngle?1:-1;
    int tolerance=1;
    if(Math.abs(servoAngle-getLastCommandedServoAngle())>tolerance){
      servoAngle+=servoDirection*Constants.Vision.SERVO_SPEED*0.02;//direction*speed*time
    }
    SmartDashboard.putNumber("Servo/CalculatedAngle", servoAngle);
  }
}
