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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
  private PhotonCamera targetCam;
  private RobotPoseEstimator robotPoseEstimator;

  private final Servo servo;
  private double servoAngle;
  private double lastTime;
  private double desiredServoAngle;

  public Vision() {
    servo = new Servo(0);
    servoAngle=getLastCommandedServoAngle();
    lastTime=Timer.getFPGATimestamp();

    this.targetCam = new PhotonCamera("target");
    updateCamAngle();
  }

  //Servo Functions
  public void setServoAngle(double angle){
    // this.desiredServoAngle=angle;
    servo.setAngle(angle);}
  // }
  public double getLastCommandedServoAngle(){return servo.getAngle();}
  public double getServoAngle(){return this.servoAngle;}

  //Cam Functions
  public boolean isConnected(){
    return targetCam.isConnected();
  }
  public boolean hasTargets(){
    return targetCam.getLatestResult().hasTargets();
  }
  //Currently getEstimatedGlobalPos is not used; instead, getEstimatedGlobalPosFromRotatingCam is used. 
  private Pair<Pose2d, Double> getEstimatedGlobalPos(Pose2d prevEstimatedRobotPose) {
    robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    double currentTime = Timer.getFPGATimestamp();
    Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
    if (result.isPresent()) {
        return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
    } else {
        return new Pair<Pose2d, Double>(null, 0.0);
    }
  }
  private void updateCamAngle(){
    Translation3d camTrans=Constants.Vision.robotToCam.getTranslation();
    Rotation3d servoAngleRot3d = new Rotation3d(0,0,servoAngle);
    Rotation3d camRot = Constants.Vision.robotToCam.getRotation().rotateBy(servoAngleRot3d);
    Transform3d robotToCam= new Transform3d(camTrans, camRot);

    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(targetCam, robotToCam));
    this.robotPoseEstimator=new RobotPoseEstimator(Constants.Vision.atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
  }
  public Pair<Pose2d, Double> getEstimatedGlobalPosFromRotatingCam(Pose2d prevEstimatedRobotPose) {
    updateCamAngle();
    return getEstimatedGlobalPos(prevEstimatedRobotPose);
  }

  @Override
  public void periodic() {

    //Servo Code
    double deltaTime=Timer.getFPGATimestamp()-lastTime;
    lastTime=Timer.getFPGATimestamp();

    int servoDirection=getLastCommandedServoAngle()>servoAngle?1:-1;
    double deltaAngle=servoDirection*Constants.Vision.SERVO_SPEED*deltaTime;//direction*speed*time

    if((servoDirection>0 && servoAngle+deltaAngle>getLastCommandedServoAngle())
     ||(servoDirection<0 && servoAngle+deltaAngle<getLastCommandedServoAngle()))
      servoAngle=getLastCommandedServoAngle();
    else
      servoAngle+=deltaAngle;
    
    SmartDashboard.putNumber("Vision/lastCommandedServoAngle", getLastCommandedServoAngle());
    SmartDashboard.putNumber("Vision/servoAngle", servoAngle);
  }
}
