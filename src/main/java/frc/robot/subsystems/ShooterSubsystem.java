package frc.robot.subsystems;

import java.lang.reflect.Method;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

   PIDController pid = new PIDController(0.1, 0.1, 0.1);
   private static final double setPoint = 0.5; // FIXME

   public CANSparkMax shooter2_motor = new CANSparkMax(Constants.ShooterConstants.SHOOTER_2_MOTOR_PORT_ID,
         MotorType.kBrushless);
   public CANSparkMax shooter1_motor = new CANSparkMax(Constants.ShooterConstants.SHOOTER_1_MOTOR_PORT_ID,
         MotorType.kBrushless);
   public CANSparkMax elevator_motor = new CANSparkMax(Constants.ShooterConstants.ELEVATOR_MOTOR_PORT_ID,
         MotorType.kBrushless);
   public CANSparkMax turret_motor = new CANSparkMax(Constants.ShooterConstants.TURRET_MOTOR_PORT_ID,
         MotorType.kBrushless);

   public RelativeEncoder shooter_1() {
      return shooter1_motor.getEncoder();
   }

   public RelativeEncoder shooter_2() {
      return shooter2_motor.getEncoder();
   }

   public RelativeEncoder turret() {
      return turret_motor.getEncoder();
   }

   public RelativeEncoder elevator() {
      return elevator_motor.getEncoder();
   }

   public RelativeEncoder s_encoder = shooter_1();
   public RelativeEncoder s_encoder2 = shooter_2();
   public RelativeEncoder t_encoder = turret();
   public RelativeEncoder e_encoder = elevator();

   double s_speed = pid.calculate(s_encoder.getPosition(), setPoint);
   double e_speed = pid.calculate(e_encoder.getPosition(), setPoint);
   double t_speed = pid.calculate(t_encoder.getPosition(), setPoint);

   public ShooterSubsystem() {
      shooter2_motor.follow(shooter1_motor);

   }

}