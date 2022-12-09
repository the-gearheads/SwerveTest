package frc.robot.subsystems.drive.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Robot;
import frc.robot.Constants.Drivetrain;

public class NEODrive implements DriveMotor {
  CANSparkMax motor;
  RelativeEncoder encoder;
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Drivetrain.DRIVE_KS, Drivetrain.DRIVE_KV, Drivetrain.DRIVE_KA);

  public NEODrive(int id, boolean invertSteer) {
    /* SparkMAX is honestly really nice and simple */
    motor = new CANSparkMax(id, MotorType.kBrushless);
    if(!Robot.isReal()) {
      REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1));
    }
    encoder = motor.getEncoder();
    /**
     * Very condensed version of this:
     * First. we use the gear ratio in reverse by multiplying by 1/DRIVE_GEAR_RATIO. This gives us the wheel rotation count.
     * Then, we multiple by the circumference to get distrance traveled (in meters).
    */
    double conversionFactor = Drivetrain.WHEEL_CIRCUMFERENCE / Drivetrain.DRIVE_GEAR_RATIO;
    encoder.setPositionConversionFactor(conversionFactor);
    encoder.setVelocityConversionFactor(conversionFactor * (1/60.0));
    motor.setInverted(invertSteer);
    setBrakeMode(true);
  } 

  public void setBrakeMode(boolean isBraking) {
    if (isBraking) {
      motor.setIdleMode(IdleMode.kBrake);
    } else {
      motor.setIdleMode(IdleMode.kCoast);
    }
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public void setSpeed(double speed) {
    double voltage = feedforward.calculate(speed);
    setVoltage(voltage);
  }

  public void zeroEncoders() {
    encoder.setPosition(0);
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public double getVelocity() {
    return Robot.isReal() ? encoder.getVelocity() : encoder.getVelocity() * (Drivetrain.WHEEL_CIRCUMFERENCE / Drivetrain.DRIVE_GEAR_RATIO);
  }

}
