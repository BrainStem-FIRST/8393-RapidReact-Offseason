package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultShooterCommand extends CommandBase{
    private ShooterSubsystem shooterSubsystem;
    private double shooterSpeed;
    private double turretSpeed;
    private double elevatorSpeed;
    public DefaultShooterCommand(ShooterSubsystem shooterSubsystem, double shooterSpeed, double turretSpeed, double elevatorSpeed){
        this.shooterSubsystem = shooterSubsystem;
        this.turretSpeed = turretSpeed;
        this.elevatorSpeed = elevatorSpeed;
        this.shooterSpeed = shooterSpeed;
        addRequirements(shooterSubsystem);
    }


    @Override
    public void initialize(){
        shooterSubsystem.initializeAllMotors();
    }

    @Override
    public void execute(){
        shooterSubsystem.executeAllMotors(shooterSpeed, turretSpeed, elevatorSpeed);
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.endAllMotors();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
