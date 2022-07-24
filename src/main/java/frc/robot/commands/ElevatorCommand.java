package frc.robot.commands;

import java.lang.reflect.Method;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


public class ElevatorCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
 private double elevatorSpeed;
   

    public ElevatorCommand(ShooterSubsystem shooterSubsystem, double elevatorSpeed){
        this.shooterSubsystem = shooterSubsystem;
        this.elevatorSpeed = elevatorSpeed;
        
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.stopElevatorMotor();
        shooterSubsystem.resetElevatorMotorEncoder();
    }

    @Override
    public void execute(){
       shooterSubsystem.setElevatorSpeed();
    }

    @Override 
    public void end(boolean interrupted){
       shooterSubsystem.stopElevatorMotor();
       shooterSubsystem.close();
    }

    @Override 
    public boolean isFinished(){
        return false;
    }
    
}
