package frc.robot.commands;

import java.lang.reflect.Method;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;


public class ElevatorCommand extends CommandBase{
    private ShooterSubsystem shooterSubsystem;
    private double speed;

    public ElevatorCommand(ShooterSubsystem shooterSubsystem, double speed){
        this.shooterSubsystem = shooterSubsystem;
        this.speed = speed;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.e_encoder.setPosition(0);
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