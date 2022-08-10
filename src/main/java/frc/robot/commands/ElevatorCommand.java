package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ElevatorCommand extends CommandBase{
    private ShooterSubsystem shooterSubsystem;
    private double elevatorSetPoint;
    public ElevatorCommand(ShooterSubsystem shooterSubsystem, double elevatorSetPoint){
        this.shooterSubsystem = shooterSubsystem;
        this.elevatorSetPoint = elevatorSetPoint;
        addRequirements(shooterSubsystem);
    }


    @Override
    public void initialize(){
        shooterSubsystem.initElevatorMotor();
    }

    @Override
    public void execute(){
        shooterSubsystem.executeElevatorMotor(elevatorSetPoint);
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.endElevator();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
