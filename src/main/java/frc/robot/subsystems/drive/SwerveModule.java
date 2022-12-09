package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.motors.CIMSteer;
import frc.robot.subsystems.drive.motors.NEODrive;

public class SwerveModule {

  private NEODrive drive;
  public CIMSteer steer;
  
  public int id;
  public String description;
  public String folderName = "";
  private Rotation2d targetAngle = new Rotation2d();
  private double targetSpeed = 0;

  private FlywheelSim rotSim, driveSim;

  SwerveModule(int driveId, int steerId, Rotation2d angleOffset, String description, boolean invertSteer) {
    folderName = "Wheel " + id + " (" + this.description + ")";
    drive = new NEODrive(driveId, !invertSteer);
    steer = new CIMSteer(steerId, angleOffset, folderName);
    rotSim = steer.getSim();
    driveSim = drive.getSim();
    id = driveId;
    this.description = description;
  }

  public double getAngle() {
    return steer.getAngle();
  }
  
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getAngle());
  }

  public double getPosition() {
    return drive.getPosition();
  }

  public double getVelocity() {
    return drive.getVelocity();
  }

  public void zeroEncoders() {
    drive.zeroEncoders();
    steer.zeroEncoders();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), getRotation2d());
  }

  public void setState(SwerveModuleState state) {
    targetSpeed = state.speedMetersPerSecond;
    targetAngle = state.angle;
  }

  public void setPIDConstants(double kF, double kP, double kI, double kD){
    steer.setPIDConstants(kF, kP, kI, kD);
  }
  
  public void periodic() {
    if(SmartDashboard.getBoolean("/Swerve/CoolWheelStuff", true)) {
      steer.setAngleMod360(targetAngle.getDegrees());
    } else {
      steer.setAngle(targetAngle.getDegrees());
    }
    SmartDashboard.putNumber("/Swerve/Wheel " + folderName + "/TargetAngle", targetAngle.getDegrees());
    SmartDashboard.putNumber("/Swerve/Wheel " + folderName + "/CurrentAngle", getRotation2d().getDegrees());

    SmartDashboard.putNumber("/Swerve/Wheel " + folderName + "/CurrentAngleModulo360", getRotation2d().getDegrees()%360);
    SmartDashboard.putNumber("/Swerve/Wheel " + folderName + "/TargetSpeed", targetSpeed);

    SmartDashboard.putNumber("/Swerve/Wheel " + folderName + "/DrivePos", drive.getPosition());
    SmartDashboard.putNumber("/Swerve/Wheel " + folderName + "/DriveVel", drive.getVelocity());

  }

  private double simRotDistance = 0;
  private double simDriveDistance = 0;

  public void simPeriodic(double dt) {
    // In theory, sparkmax sim should be handled roughly by the RevPhysicsSim thing, so we need to simulate
    // the turn encoder ourselves
    rotSim.setInputVoltage(steer.getAppliedVolts());
    driveSim.setInputVoltage(drive.getAppliedVolts());
    rotSim.update(dt);
    driveSim.update(dt);

    simRotDistance += rotSim.getAngularVelocityRPM() * 360 * dt;
    steer.setSimPosition(simRotDistance);
    steer.setSimVelocity(rotSim.getAngularVelocityRPM() * 360);

    simDriveDistance += driveSim.getAngularVelocityRPM() * drive.conversionFactor * dt;
    drive.simSetPosition(simDriveDistance);
    drive.simSetVelocity(driveSim.getAngularVelocityRPM() * drive.conversionFactor);

  }


}
