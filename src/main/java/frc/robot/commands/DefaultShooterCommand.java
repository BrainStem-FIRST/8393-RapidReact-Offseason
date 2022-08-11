package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultShooterCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;

    private DoubleSupplier shooterSpeed;
    private DoubleSupplier elevatorSpeed;
    private DoubleSupplier turretSpeed;
    private double triggerThreshold;
    public boolean isAuto;

    public DefaultShooterCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier shootingSpeed, DoubleSupplier elevatorSpeed, DoubleSupplier turretSpeed, double triggerThreshold, boolean isAuto){
        this.shooterSubsystem = shooterSubsystem;
        this.shooterSpeed = shootingSpeed;
        this.elevatorSpeed = elevatorSpeed;
        this.turretSpeed = turretSpeed;
        this.triggerThreshold = triggerThreshold;
        this.isAuto = isAuto;

        addRequirements(shooterSubsystem);
    }


    


    @Override
    public void initialize(){
        shooterSubsystem.initAllMotors();
    }

    @Override
    public void execute(){
        double shooterSpeedDouble = Math.abs(shooterSpeed.getAsDouble()) > triggerThreshold ? ShooterConstants.SHOOTING_MOTORS_SPEED : 0.0;
        double elevatorSpeedDouble = elevatorSpeed.getAsDouble();
        double turretSpeedDouble = turretSpeed.getAsDouble();
        if(isAuto) {
            shooterSubsystem.executeAllMotorsAuto(shooterSpeedDouble, elevatorSpeedDouble, turretSpeedDouble);
        } else {
            shooterSubsystem.executeAllMotors(shooterSpeedDouble, elevatorSpeedDouble, turretSpeedDouble);
        }
    }

    boolean programmedWell = true;
    String isMihirHappy =  programmedWell ? "Yes" : "No";

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.endAllMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
