package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
//import frc.robot.Constants.ColorSensorConstants;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {

   /*
    * private final I2C.Port ColorSensorI2C = I2C.Port.kOnboard;
    * private final ColorSensorV3 transferColorSensor = new
    * ColorSensorV3(ColorSensorI2C);
    * private final int currentRValue = transferColorSensor.getRed();
    * private final int currentGValue = transferColorSensor.getGreen();
    * private final int currentBValue = transferColorSensor.getBlue();
    */

   Alliance alliance = DriverStation.getAlliance();
   PIDController turretPIDController = new PIDController(ShooterConstants.TURRET_PROPORTIONAL,
         ShooterConstants.TURRET_INTREGRAL,
         ShooterConstants.TURRET_DERIVATIVE);

   PIDController elevatorPIDController = new PIDController(ShooterConstants.ELEVATOR_PROPORTIONAL,
         ShooterConstants.ELEVATOR_INTEGRAL,
         ShooterConstants.ELEVATOR_DERIVATIVE);

   PIDController shooterPIDController = new PIDController(ShooterConstants.SHOOTER_PROPORTIONAL,
         ShooterConstants.SHOOTER_INTEGRAL,
         ShooterConstants.SHOOTER_DERIVATIVE);

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

   public void setShooterSpeed(double desiredSpeed) {

      shooterPIDController.setTolerance(ShooterConstants.SHOOTER_PID_TOLERANCE);
      double shooterSpeed = shooterPIDController.calculate(shooterMotor1Encoder.getVelocity(), desiredSpeed);
      shooterMotor2.follow(shooterMotor1);
      shooterMotor1.set(shooterSpeed);
   }

   public void setTurretSpeed(double setPoint) {
      turretPIDController.setTolerance(ShooterConstants.TURRET_PID_TOLERANCE);
      double turretSpeed = turretPIDController.calculate(turretMotorEncoder.getPosition(), setPoint);
      turretMotor.set(turretSpeed);
   }

   public void setElevatorSpeed(double setPoint) {
      elevatorPIDController.setTolerance(ShooterConstants.ELEVATOR_PID_TOLERANCE);
      double elevatorSpeed = elevatorPIDController.calculate(elevatorMotorEncoder.getPosition(), setPoint);
      elevatorMotor.set(elevatorSpeed);
   }

   public void setElevatorToRemoveFreight(double setPoint) {
      elevatorPIDController.setTolerance(ShooterConstants.ELEVATOR_PID_TOLERANCE);
      double elevatorSpeed = elevatorPIDController.calculate(elevatorMotorEncoder.getPosition(),
            setPoint);
      elevatorMotor.set(elevatorSpeed);
   }

   public double elevatorCurrentPos() {
      return elevatorMotorEncoder.getPosition();
   }

   public double turretCurrentPos() {
      return turretMotorEncoder.getPosition();
   }

   public void initShooter() {
      stopShooterMotors();
   }

   public void initTurret() {
      resetTurretMotorEncoder();
      stopTurretMotor();
   }

   public void initElevator() {
      resetElevatorMotorEncoder();
      stopShooterMotors();
   }

   public void executeShooter(double desiredSpeed) {
      setShooterSpeed(desiredSpeed);
   }

   public void executeTurret(double turretSetPoint) {
      setTurretSpeed(turretSetPoint);
   }

   /*public void executeElevator(double elevatorSetPoint) {
      if ((alliance == DriverStation.Alliance.Blue && isCargoBlue())
            || (alliance == DriverStation.Alliance.Red && isCargoRed())) {
         setElevatorSpeed(elevatorSetPoint);
      } else {
         setElevatorToRemoveFreight(elevatorSetPoint);
      }
   }*/

   public void endShooter() {
      stopShooterMotors();
   }

   public void endElevator() {
      stopElevatorMotor();
      resetElevatorMotorEncoder();
   }

   public void endTurret() {
      stopTurretMotor();
      resetTurretMotorEncoder();
   }
   /*
    * public boolean isCargoBlue() {
    * return ( (ColorSensorConstants.BLUE_ALLIANCE_R_VALUE < currentRValue) &&
    * (ColorSensorConstants.BLUE_ALLIANCE_G_VALUE < currentGValue) &&
    * (ColorSensorConstants.BLUE_ALLIANCE_B_VALUE < currentBValue));
    * }
    * public boolean isCargoRed() {
    * return ( (ColorSensorConstants.RED_ALLIANCE_R_VALUE < currentRValue) &&
    * (ColorSensorConstants.RED_ALLIANCE_G_VALUE < currentGValue) &&
    * (ColorSensorConstants.RED_ALLIANCE_B_VALUE < currentBValue));
    * }
    */

   public void resetBothShooterMotorEncoders() {
      shooterMotor1Encoder.setPosition(0);
      shooterMotor2Encoder.setPosition(0);
   }

   public void resetElevatorMotorEncoder() {
      elevatorMotorEncoder.setPosition(0);
   }

   public void resetTurretMotorEncoder() {
      turretMotorEncoder.setPosition(0);
   }

   public void resetAllMotorEncoders() {
      resetElevatorMotorEncoder();
      resetBothShooterMotorEncoders();
      resetTurretMotorEncoder();
   }

   public void stopElevatorMotor() {
      elevatorMotor.set(0);
   }

   public void stopShooterMotors() {
      shooterMotor1.set(0);
      shooterMotor2.set(0);
   }

   public void stopTurretMotor() {
      turretMotor.set(0);
   }

   public void stopAllMotors() {
      stopShooterMotors();
      stopTurretMotor();
      stopElevatorMotor();
   }

   @Override
   public void close() throws Exception {
      shooterMotor1.close();
      shooterMotor2.close();
      turretMotor.close();
      elevatorMotor.close();
   }

}