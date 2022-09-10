package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ConstraintsConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {

   private PIDController shooterPIDController = new PIDController(ShooterConstants.SHOOTER_PROPORTIONAL,
         ShooterConstants.SHOOTER_INTEGRAL,
         ShooterConstants.SHOOTER_DERIVATIVE);

   private CANSparkMax shooterMotor1 = new CANSparkMax(Constants.ShooterConstants.SHOOTER_1_MOTOR_PORT_ID,
         MotorType.kBrushless);

   private CANSparkMax shooterMotor2 = new CANSparkMax(Constants.ShooterConstants.SHOOTER_2_MOTOR_PORT_ID,
         MotorType.kBrushless);

   public RelativeEncoder shooterMotor1Encoder = returnShooterMotor1Encoder();
   public RelativeEncoder shooterMotor2Encoder = returnShooterMotor2Encoder();

   public RelativeEncoder returnShooterMotor1Encoder() {
      return shooterMotor1.getEncoder();
   }

   public RelativeEncoder returnShooterMotor2Encoder() {
      return shooterMotor2.getEncoder();
   }

   public void setShooterSpeed(double speed, boolean usePID) {
      double updatedSpeed;
      if (usePID) {
         shooterPIDController.setTolerance(ShooterConstants.SHOOTER_PID_TOLERANCE);
         updatedSpeed = ShooterConstants.SHOOTING_MOTORS_REVERSED ? shooterPIDController
               .calculate(shooterMotor1Encoder.getVelocity() /
                     ConstraintsConstants.CAN_SPARK_MAX_MAXIMUM_RPM, -speed)
               : shooterPIDController
                     .calculate(shooterMotor1Encoder.getVelocity() /
                           ConstraintsConstants.CAN_SPARK_MAX_MAXIMUM_RPM, speed);
      } else {
         updatedSpeed = ShooterConstants.SHOOTING_MOTORS_REVERSED ? -speed : speed;
      }
      shooterMotor2.follow(shooterMotor1);
      shooterMotor1.set(updatedSpeed);
   }

   public void initializeShooterMotors() {
      resetShooterMotorEncoders();
      stopShooterMotors();
   }

   public void executeShooterMotors(double shooterspeed, boolean usePID) {
      setShooterSpeed(shooterspeed, usePID);
   }

   public void endShooterMotors() {
      initializeShooterMotors();
   }

   public void resetShooterMotorEncoders() {
      shooterMotor1Encoder.setPosition(0);
      shooterMotor2Encoder.setPosition(0);
   }

   public void stopShooterMotors() {
      shooterMotor1.set(0);
      shooterMotor2.set(0);
   }

   @Override
   public void close() throws Exception {
      shooterMotor1.close();
      shooterMotor2.close();
   }

}