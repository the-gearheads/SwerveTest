// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  private PhotonCamera targetCam;
  private RobotPoseEstimator robotPoseEstimator;

  public Vision() {

    this.targetCam = new PhotonCamera("target");
    updateCamAngle(0);
  }

  //Cam Functions
  public boolean isConnected(){
    return targetCam.isConnected();
  }
  public boolean hasTargets(){
    return targetCam.getLatestResult().hasTargets();
  }
  //Currently getEstimatedGlobalPosFromRotating CAM is not used; instead, getEstimatedGlobalPos is used. 
  public Pair<Pose2d, Double> getEstimatedGlobalPosFromRotatingCam(Pose2d prevEstimatedRobotPose, double servoAngle) {
    updateCamAngle(servoAngle);
    return getEstimatedGlobalPos(prevEstimatedRobotPose);
  }
  public Pair<Pose2d, Double> getEstimatedGlobalPos(Pose2d prevEstimatedRobotPose) {
    robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    double currentTime = Timer.getFPGATimestamp();
    Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
    if (result.isPresent() && result.get().getFirst()!=null&&result.get().getFirst().getRotation()!=null) {//the 2nd and 3rd conditions avoid nullpointer exceptions. 
      SmartDashboard.putBoolean("Vision/ResultPresent", true);
        return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
    } else {
        SmartDashboard.putBoolean("Vision/ResultPresent", false);
        return new Pair<Pose2d, Double>(prevEstimatedRobotPose, 0.0);
    }
  }
  private void updateCamAngle(double servoAngle){
    Translation3d camTrans=Constants.Vision.robotToCam.getTranslation();
    Rotation3d servoAngleRot3d = new Rotation3d(0,0,servoAngle);
    Rotation3d camRot = Constants.Vision.robotToCam.getRotation().rotateBy(servoAngleRot3d);
    Transform3d robotToCam= new Transform3d(camTrans, camRot);

    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(targetCam, robotToCam));
    this.robotPoseEstimator=new RobotPoseEstimator(Constants.Vision.atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
  }

  @Override
  public void periodic() {
  }
}
