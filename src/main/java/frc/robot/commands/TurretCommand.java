package frc.robot.commands;

import java.lang.reflect.Method;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


public class TurretCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
 private double turretSetPoint;
   

    public TurretCommand(ShooterSubsystem shooterSubsystem, double goToThisPosition){
        this.shooterSubsystem = shooterSubsystem;
        this.turretSetPoint = goToThisPosition;
        
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.stopTurretMotor();
        shooterSubsystem.resetTurretMotorEncoder();
    }

    @Override
    public void execute(){
       shooterSubsystem.setTurretSpeed(turretSetPoint);
    }

    @Override 
    public void end(boolean interrupted){
       shooterSubsystem.stopTurretMotor();
       shooterSubsystem.resetTurretMotorEncoder();
    }

    @Override 
    public boolean isFinished(){
        return false;
    }
    
}