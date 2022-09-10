package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstraintsConstants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase implements AutoCloseable {

    private CANSparkMax turretMotor = new CANSparkMax(TurretConstants.TURRET_MOTOR_PORT_ID,
            MotorType.kBrushless);

    private PIDController turretPIDController = new PIDController(TurretConstants.TURRET_PROPORTIONAL,
            TurretConstants.TURRET_INTREGRAL,
            TurretConstants.TURRET_DERIVATIVE);

    private RelativeEncoder turretMotorEncoder = returnTurretMotorEncoder();

    public RelativeEncoder returnTurretMotorEncoder() {
        return turretMotor.getEncoder();
    }

    public void setTurretSpeed(double speed, boolean usePID) {
        double updatedSpeed;
        if (usePID) {
            turretPIDController.setTolerance(TurretConstants.TURRET_PID_TOLERANCE);
            updatedSpeed = TurretConstants.TURRET_MOTOR_REVERSED
                    ? turretPIDController.calculate(turretMotorEncoder.getVelocity() /
                            ConstraintsConstants.CAN_SPARK_MAX_MAXIMUM_RPM, -speed)
                    : turretPIDController
                            .calculate(turretMotorEncoder.getVelocity() /
                                    ConstraintsConstants.CAN_SPARK_MAX_MAXIMUM_RPM, speed);
            updatedSpeed = TurretConstants.TURRET_MOTOR_REVERSED ? -speed : speed;
        } else {
            updatedSpeed = TurretConstants.TURRET_MOTOR_REVERSED ? -speed : speed;
        }
        turretMotor.set(updatedSpeed);
    }

    public double getTurretPosition() {
        return turretMotorEncoder.getPosition();
    }

    public void initializeTurretMotor() {
        resetTurretMotorEncoder();
        stopTurretMotor();
    }

    public void executeTurretMotor(double turretPower, boolean usePID) {
        setTurretSpeed(turretPower, usePID);
    }

    public void endTurret() {
        initializeTurretMotor();
    }

    public void resetTurretMotorEncoder() {
        turretMotorEncoder.setPosition(0);
    }

    public void stopTurretMotor() {
        turretMotor.set(0.0);
    }

    @Override
    public void close() {
        turretMotor.close();
    }
}
