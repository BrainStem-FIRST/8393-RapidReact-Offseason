package frc.robot.commands;

import java.lang.reflect.Method;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


public class ShooterCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    public double s_speed;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, double s_speed){
        this.shooterSubsystem = shooterSubsystem;
        this.s_speed = s_speed;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.s_encoder.setPosition(0);
        shooterSubsystem.s_encoder2.setPosition(0);
    }

    @Override
    public void execute(){
        
        shooterSubsystem.shooter1_motor.set(s_speed);

    }

    @Override 
    public void end(boolean interrupted){
        

    }

    @Override 
    public boolean isFinished(){
        return false;
    }
    
}