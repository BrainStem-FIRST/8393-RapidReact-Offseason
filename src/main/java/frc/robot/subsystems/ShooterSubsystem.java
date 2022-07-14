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

   public RelativeEncoder returnShooterMotor2Encoder() {
      return shooterMotor1.getEncoder();
   }

   public RelativeEncoder returnShooterMotor1Encoder() {
      return shooterMotor2.getEncoder();
   }

   public RelativeEncoder returnTurretMotorEncoder() {
      return turretMotor.getEncoder();
   }

   public RelativeEncoder returnElevatorMotorEncoder() {
      return elevatorMotor.getEncoder();
   }

   public void setShooterSpeed(double speed) {
      shooterMotor2.follow(shooterMotor1);
      shooterMotor1.set(speed);
   }

   public void setTurretSpeed(double speed){
      turretMotor.set(speed);
   }

   public void setElevatorSpeed(double speed){
      elevatorMotor.set(speed);
   }

   public void setAllMotorSpeeds(double shooterSpeed, double elevatorSpeed, double turretSpeed){
      setShooterSpeed(shooterSpeed);
      setElevatorSpeed(elevatorSpeed);
      setTurretSpeed(turretSpeed);
   }
   public void resetAllMotorEncoders() {
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
      resetAllMotorEncoders();
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

   double shooterSpeed = pidController.calculate(shooterMotor1Encoder.getPosition(), ShooterConstants.SET_PID_LOCATION);
   double elevatorSpeed = pidController.calculate(elevatorMotorEncoder.getPosition(), ShooterConstants.SET_PID_LOCATION);
   double turretSpeed = pidController.calculate(turretMotorEncoder.getPosition(), ShooterConstants.SET_PID_LOCATION);

   @Override
   public void close() {
      shooterMotor1.close();
      shooterMotor2.close();
      elevatorMotor.close();
      turretMotor.close();
   }
}