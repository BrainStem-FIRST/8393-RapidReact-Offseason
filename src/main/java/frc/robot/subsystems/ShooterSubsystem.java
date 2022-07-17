package frc.robot.subsystems;

import java.lang.reflect.Method;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {

   PIDController pidController = new PIDController(ShooterConstants.PROPORTIONAL, ShooterConstants.INTREGRAL,
         ShooterConstants.DERIVATIVE);

   private CANSparkMax shooterMotor2 = new CANSparkMax(Constants.ShooterConstants.SHOOTER_2_MOTOR_PORT_ID,
         MotorType.kBrushless);
   private CANSparkMax shooterMotor1 = new CANSparkMax(Constants.ShooterConstants.SHOOTER_1_MOTOR_PORT_ID,
         MotorType.kBrushless);
   private CANSparkMax elevatorMotor = new CANSparkMax(Constants.ShooterConstants.ELEVATOR_MOTOR_PORT_ID,
         MotorType.kBrushless);
   private CANSparkMax turretMotor = new CANSparkMax(Constants.ShooterConstants.TURRET_MOTOR_PORT_ID,
         MotorType.kBrushless);

   public RelativeEncoder shooterMotor1Encoder = returnShooterMotor1Encoder();
   public RelativeEncoder shooterMotor2Encoder = returnShooterMotor2Encoder();
   public RelativeEncoder turretMotorEncoder = returnTurretMotorEncoder();
   public RelativeEncoder elevatorMotorEncoder = returnElevatorMotorEncoder();

  
public RelativeEncoder returnShooterMotor1Encoder() {
   return shooterMotor1.getEncoder();
}
public RelativeEncoder returnShooterMotor2Encoder() {
   return shooterMotor2.getEncoder();
} 


   public RelativeEncoder returnTurretMotorEncoder() {
      return turretMotor.getEncoder();
   }

   public RelativeEncoder returnElevatorMotorEncoder() {
      return elevatorMotor.getEncoder();
   }

   public void setShooterSpeed() {
      pidController.setTolerance(3);
      double shooterSpeed = pidController.calculate(shooterMotor1Encoder.getPosition(), 500);
      shooterMotor1.set(shooterSpeed);
      shooterMotor2.follow(shooterMotor1);
   
   }

   public void setTurretSpeed(){
      pidController.setTolerance(3);
      double turretSpeed = pidController.calculate(turretMotorEncoder.getPosition(), 500);
      turretMotor.set(turretSpeed); 
   }

   public void setElevatorSpeed(){
      pidController.setTolerance(3);
  double elevatorSpeed = pidController.calculate(elevatorMotorEncoder.getPosition(), 500);
      elevatorMotor.set(elevatorSpeed);
   }

  
   public void resetAllShooterMotorEncoders() {
      shooterMotor1Encoder.setPosition(0);
      shooterMotor2Encoder.setPosition(0);
   }

   public void resetElevatorMotorEncoder(){
      elevatorMotorEncoder.setPosition(0);
   }

   public void resetTurretMotorEncoder(){
      turretMotorEncoder.setPosition(0);
   }

   public void resetAllShooterEncoders(){
      resetElevatorMotorEncoder();
      resetAllShooterMotorEncoders();
      resetTurretMotorEncoder();
   }

   public void stopElevatorMotor(){
      elevatorMotor.set(0);
   }

   public void stopShooterMotors(){
      shooterMotor1.set(0);
      shooterMotor2.set(0);
   }

   public void stopTurretMotor(){
      turretMotor.set(0);
   }

   public void stopAllMotors(){
      stopShooterMotors();
      stopTurretMotor();
      stopElevatorMotor();
   }
   @Override
   public void close() {
      shooterMotor1.close();
      shooterMotor2.close();
      turretMotor.close();
      elevatorMotor.close();
   }
   
   
   


  

  
  
}