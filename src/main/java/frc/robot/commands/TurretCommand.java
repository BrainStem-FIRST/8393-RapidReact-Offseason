package frc.robot.commands;

import java.lang.reflect.Method;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


public class TurretCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
 private double elevatorSpeed;
   

    public TurretCommand(ShooterSubsystem shooterSubsystem, double turretSpeed){
        this.shooterSubsystem = shooterSubsystem;
        this.turretSpeed = turretSpeed;
        
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.stopTurretMotor();
        shooterSubsystem.resetTurretMotorEncoder();
    }

    @Override
    public void execute(){
       shooterSubsystem.setTurretSpeed();
    }

    @Override 
    public void end(boolean interrupted){
       shooterSubsystem.stopTurretMotor();
    }

    @Override 
    public boolean isFinished(){
        return false;
    }
    
}