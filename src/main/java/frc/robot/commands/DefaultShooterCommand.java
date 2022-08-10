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
    public boolean isAuto;

    public DefaultShooterCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier shootingSpeed, DoubleSupplier elevatorSpeed, DoubleSupplier turretSpeed, boolean isAuto){
        this.shooterSubsystem = shooterSubsystem;
        this.shooterSpeed = shootingSpeed;
        this.elevatorSpeed = elevatorSpeed;
        this.turretSpeed = turretSpeed;
        this.isAuto = isAuto;

        addRequirements(shooterSubsystem);
    }


    


    @Override
    public void initialize(){
        shooterSubsystem.initAllMotors();
    }

    @Override
    public void execute(){
        double shooterSpeedDouble = shooterSpeed.getAsDouble();
        double elevatorSpeedDouble = elevatorSpeed.getAsDouble();
        double turretSpeedDouble = turretSpeed.getAsDouble();
        if(isAuto) {
        shooterSubsystem.executeAllMotors(shooterSpeedDouble, elevatorSpeedDouble, turretSpeedDouble);
        } else {
            shooterSubsystem.executeAllMotorsAuto(shooterSpeedDouble, elevatorSpeedDouble, turretSpeedDouble);
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
