package frc.robot.commands;

import java.lang.reflect.Method;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


public class TurretCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
 private double turretSetPoint;
   

    public TurretCommand(ShooterSubsystem shooterSubsystem, double turretSetPoint){
        this.shooterSubsystem = shooterSubsystem;
        this.turretSetPoint = turretSetPoint;
        
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.initTurret();
    }

    @Override
    public void execute(){
       shooterSubsystem.executeTurret(turretSetPoint);
    }

    @Override 
    public void end(boolean interrupted){
       shooterSubsystem.endTurret();
    }

    @Override 
    public boolean isFinished(){
        return false;
    }
    
}