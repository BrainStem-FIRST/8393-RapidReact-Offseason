package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultShooterCommand extends CommandBase{
    private ShooterSubsystem shooterSubsystem;
    private DoubleSupplier shooterSpeed;
    private DoubleSupplier elevatorSpeed;
    private DoubleSupplier turretSpeed;

    public DefaultShooterCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier d, DoubleSupplier e, DoubleSupplier f){
        this.shooterSubsystem = shooterSubsystem;
        this.shooterSpeed = d;
        this.elevatorSpeed = e;
        this.turretSpeed = f;

        addRequirements(shooterSubsystem);
    }


    


    @Override
    public void initialize(){
        shooterSubsystem.initAllMotors();
    }

    @Override
    public void execute(){
        double shooterSpeedDouble = shooterSpeed.getAsDouble();
        double elevatorSpeedDouble = elevatorSpeed.getAsDouble();
        double turretSpeedDouble = turretSpeed.getAsDouble();
        shooterSubsystem.executeAllMotors(shooterSpeedDouble, elevatorSpeedDouble, turretSpeedDouble);
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
