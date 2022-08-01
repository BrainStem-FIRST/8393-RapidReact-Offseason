package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ConstraintsConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModuleSubsystem implements AutoCloseable {
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
    private final TalonFXSensorCollection driveMotorEncoder;
    private final TalonFXSensorCollection turningMotorEncoder;
    private final PIDController turningPIDController;
    private final CANCoder turningMotorAbsoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Swerve module");

    public SwerveModuleSubsystem(int driveMotorID, int turningMotorID, boolean driveMotorReversed,
            boolean turningMotorReversed,
            int absoluteEncoderID, double absoluteEncoderOffsetRad, boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        // creating motors
        this.driveMotor = new TalonFX(driveMotorID);
        this.turningMotor = new TalonFX(turningMotorID);
        // creating encoders
        this.turningMotorAbsoluteEncoder = new CANCoder(absoluteEncoderID);
        this.driveMotorEncoder = driveMotor.getSensorCollection();
        this.turningMotorEncoder = turningMotor.getSensorCollection();

        this.turningPIDController = new PIDController(SwerveModuleConstants.PROPORTIONAL,
                SwerveModuleConstants.INTEGRAL,
                SwerveModuleConstants.DERIVATIVE);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        resetEncoders();
    }

    public double getAbsoluteAngle() {
        double angle = Math.toRadians(turningMotorAbsoluteEncoder.getAbsolutePosition());
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }
        angle -= Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public double getDriveMotorPosition() {
        return driveMotorEncoder.getIntegratedSensorPosition() * SwerveModuleConstants.DRIVE_ENCODER_TICKS_TO_METERS;
    }

    public double getTurningMotorPosition() {
        return turningMotorEncoder.getIntegratedSensorPosition() * SwerveModuleConstants.DRIVE_ENCODER_TICKS_TO_METERS;
    }

    public double getDriveMotorVelocity() {
        return driveMotorEncoder.getIntegratedSensorVelocity()
                * SwerveModuleConstants.DRIVE_ENCODER_TICKS_TO_METERS_PER_SECOND;
    }

    public double getTurningMotorVelocity() {
        return turningMotorEncoder.getIntegratedSensorVelocity()
                * SwerveModuleConstants.TURNING_ENCODER_TICKS_TO_METERS_PER_SECOND;
    }

    public double getAbsoluteEncoderRadians() {
        // RETURNS A VALUE BETWEEN NEGATIVE PI AND PI
        return getAbsoluteAngle();
    }

    public void resetEncoders() {
        driveMotorEncoder.setIntegratedSensorPosition(0, 100);
        turningMotorEncoder.setIntegratedSensorPosition(turningMotorAbsoluteEncoder.getAbsolutePosition(), 100);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getTurningMotorPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if ((Math.abs(state.speedMetersPerSecond)) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput,
                (state.speedMetersPerSecond / ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND));
        turningMotor.set(ControlMode.PercentOutput,
                turningPIDController.calculate(getTurningMotorPosition(), state.angle.getRadians()));
    }

    public void setTurningMotorSpeed(double turningMotorSpeed){
        turningMotor.set(ControlMode.PercentOutput, 
            turningMotorSpeed);
    }

    public void setDrivingMotorSpeed(double driveMotorSpeed){
        driveMotor.set(ControlMode.PercentOutput, driveMotorSpeed);
    }

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turningMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void close() throws Exception {
    }
}
