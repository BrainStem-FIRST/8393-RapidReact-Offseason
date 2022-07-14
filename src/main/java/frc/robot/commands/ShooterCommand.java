package frc.robot.commands;

import java.lang.reflect.Method;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


public class ShooterCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    public double speed;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, double speed){
        this.shooterSubsystem = shooterSubsystem;
        this.speed = speed;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.resetShooterMotorEncoders();
    }

    @Override
    public void execute(){
        shooterSubsystem.setShooterSpeed(speed);
    }

    @Override 
    public void end(boolean interrupted){
        shooterSubsystem.stopShooterMotors();
    }

    @Override 
    public boolean isFinished(){
        return false;
    }
    
}