package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultShooterCommand extends CommandBase{
    private ShooterSubsystem shooterSubsystem;
    private double shooterSpeed;
    public DefaultShooterCommand(ShooterSubsystem shooterSubsystem, double shooterSpeed){
        this.shooterSubsystem = shooterSubsystem;
        this.shooterSpeed = shooterSpeed;
        addRequirements(shooterSubsystem);
    }


    @Override
    public void initialize(){
        shooterSubsystem.initElevatorMotor();
    }

    @Override
    public void execute(){
        shooterSubsystem.executeShooterMotors(shooterSpeed);
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.endShooter();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
