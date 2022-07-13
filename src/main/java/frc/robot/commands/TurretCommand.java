package frc.robot.commands;

import java.lang.reflect.Method;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;


public class TurretCommand extends CommandBase{
    private ShooterSubsystem shooterSubsystem;
    private double t_speed;

    public TurretCommand(ShooterSubsystem shooterSubsystem, double t_speed){
        this.shooterSubsystem = shooterSubsystem;
        this.t_speed = t_speed;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.t_encoder.setPosition(0);
    }

    @Override
    public void execute(){
        

    }

    @Override 
    public void end(boolean interrupted){
        

    }

    @Override 
    public boolean isFinished(){
        return false;
    }
    
}
