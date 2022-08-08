package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultShooterCommand extends CommandBase{
    private ShooterSubsystem shooterSubsystem;
    private double shooterSpeed;
    private double elevatorSetPoint;
    private double turretSetPoint;

    public DefaultShooterCommand(ShooterSubsystem shooterSubsystem, double shooterSpeed, double elevatorSetPoint, double turretSetPoint){
        this.shooterSubsystem = shooterSubsystem;
        this.shooterSpeed = shooterSpeed;
        this.elevatorSetPoint = elevatorSetPoint;
        this.turretSetPoint = turretSetPoint;

        addRequirements(shooterSubsystem);
    }


    @Override
    public void initialize(){
        shooterSubsystem.initAllMotors();
    }

    @Override
    public void execute(){
        shooterSubsystem.executeAllMotors(shooterSpeed, elevatorSetPoint, turretSetPoint);
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.endAllMotors();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
