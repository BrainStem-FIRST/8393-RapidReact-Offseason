package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ConstraintsConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {

   private PIDController turretPIDController = new PIDController(ShooterConstants.TURRET_PROPORTIONAL,
         ShooterConstants.TURRET_INTREGRAL,
         ShooterConstants.TURRET_DERIVATIVE);

   private PIDController elevatorPIDController = new PIDController(ShooterConstants.ELEVATOR_PROPORTIONAL,
         ShooterConstants.ELEVATOR_INTEGRAL,
         ShooterConstants.ELEVATOR_DERIVATIVE);

   private PIDController shooterPIDController = new PIDController(ShooterConstants.SHOOTER_PROPORTIONAL,
         ShooterConstants.SHOOTER_INTEGRAL,
         ShooterConstants.SHOOTER_DERIVATIVE);

   private CANSparkMax shooterMotor1 = new CANSparkMax(Constants.ShooterConstants.SHOOTER_1_MOTOR_PORT_ID,
         MotorType.kBrushless);

   private CANSparkMax shooterMotor2 = new CANSparkMax(Constants.ShooterConstants.SHOOTER_2_MOTOR_PORT_ID,
         MotorType.kBrushless);
         
   private CANSparkMax turretMotor = new CANSparkMax(Constants.ShooterConstants.TURRET_MOTOR_PORT_ID,
         MotorType.kBrushless);

   public RelativeEncoder shooterMotor1Encoder = returnShooterMotor1Encoder();
   public RelativeEncoder shooterMotor2Encoder = returnShooterMotor2Encoder();
   public RelativeEncoder turretMotorEncoder = returnTurretMotorEncoder();

   public RelativeEncoder returnShooterMotor1Encoder() {
      return shooterMotor1.getEncoder();
   }

   public RelativeEncoder returnShooterMotor2Encoder() {
      return shooterMotor2.getEncoder();
   }

   public RelativeEncoder returnTurretMotorEncoder() {
      return turretMotor.getEncoder();
   }



   public void setShooterSpeed(double speed) {
      shooterPIDController.setTolerance(ShooterConstants.SHOOTER_PID_TOLERANCE);
      /*double shooterSpeed = ShooterConstants.SHOOTING_MOTORS_REVERSED ? shooterPIDController
            .calculate(shooterMotor1Encoder.getVelocity() / ConstraintsConstants.CAN_SPARK_MAX_MAXIMUM_RPM, -speed) : 
            shooterPIDController
            .calculate(shooterMotor1Encoder.getVelocity() / ConstraintsConstants.CAN_SPARK_MAX_MAXIMUM_RPM, speed);*/
      double shooterSpeed = ShooterConstants.SHOOTING_MOTORS_REVERSED ? -speed : speed;
      shooterMotor2.follow(shooterMotor1);
      shooterMotor1.set(shooterSpeed);
   }

   public void setTurretSpeed(double speed) {
   /*
     turretPIDController.setTolerance(ShooterConstants.TURRET_PID_TOLERANCE);
      double turretSpeed = ShooterConstants.TURRET_MOTOR_REVERSED ? turretPIDController
            .calculate(turretMotorEncoder.getVelocity() / ConstraintsConstants.CAN_SPARK_MAX_MAXIMUM_RPM, -speed) : 
            turretPIDController
            .calculate(turretMotorEncoder.getVelocity() / ConstraintsConstants.CAN_SPARK_MAX_MAXIMUM_RPM, speed);
      //turretSpeed = ShooterConstants.TURRET_MOTOR_REVERSED ? -speed : speed;
      */
      turretMotor.set(speed);
   }

   
   public double turretPosition() {
      return turretMotorEncoder.getPosition();
   }

   public void initShooterMotors() {
      resetBothShooterMotorEncoders();
      stopShooterMotors();
   }

   public void initTurretMotor() {
      resetTurretMotorEncoder();
      stopTurretMotor();
   }


   public void initAllMotors() {
      initShooterMotors();
      initTurretMotor();
   }

  

   public void executeShooterMotors(double shooterspeed) {
      setShooterSpeed(shooterspeed);
   }

   public void executeTurretMotor(double turretPower) {
      setTurretSpeed(turretPower);
   }



   public void executeAllMotors(double shooterSpeed, double turretSpeed, double elevatorSpeed){
      executeShooterMotors(shooterSpeed);
      executeTurretMotor(turretSpeed);
   }

   public void endShooter() {
      initShooterMotors();
   }

 

   public void endTurret() {
      initTurretMotor();
   }
   public void endAllMotors() {
      endShooter();
      endTurret();
   }

   

   public void resetBothShooterMotorEncoders() {
      shooterMotor1Encoder.setPosition(0);
      shooterMotor2Encoder.setPosition(0);
   }


   public void resetTurretMotorEncoder() {
      turretMotorEncoder.setPosition(0);
   }

   public void resetAllMotorEncoders() {
      resetBothShooterMotorEncoders();
      resetTurretMotorEncoder();
   }

   public void stopElevatorMotor() {
   }

   public void stopShooterMotors() {
      shooterMotor1.set(0);
      shooterMotor2.set(0);
   }

   public void stopTurretMotor() {
      turretMotor.set(0);
   }

  

   @Override
   public void close() throws Exception {
      shooterMotor1.close();
      shooterMotor2.close();
      turretMotor.close();
   }
}