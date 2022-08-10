package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class TurretCommand extends CommandBase{
    private ShooterSubsystem shooterSubsystem;
    private double turretSetPoint;
    public TurretCommand(ShooterSubsystem shooterSubsystem, double turretSetPoint){
        this.shooterSubsystem = shooterSubsystem;
        this.turretSetPoint = turretSetPoint;
        addRequirements(shooterSubsystem);
    }


    @Override
    public void initialize(){
        shooterSubsystem.initElevatorMotor();
    }

    @Override
    public void execute(){
        shooterSubsystem.executeTurretMotor(turretSetPoint);
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
