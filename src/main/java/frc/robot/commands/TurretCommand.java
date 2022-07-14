package frc.robot.commands;

import java.lang.reflect.Method;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;


public class TurretCommand extends CommandBase{
    private ShooterSubsystem shooterSubsystem;
    private double turretSpeed;

    public TurretCommand(ShooterSubsystem shooterSubsystem, double turretSpeed){
        this.shooterSubsystem = shooterSubsystem;
        this.turretSpeed = turretSpeed;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.resetTurretMotorEncoder();
    }

    @Override
    public void execute(){
        shooterSubsystem.setTurretSpeed(turretSpeed);
    }

    @Override 
    public void end(boolean interrupted){
    }

    @Override 
    public boolean isFinished(){
        return false;
    }
    
}
