package frc.robot.commands;

import java.lang.reflect.Method;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    private double shooterSpeed;
    

    public ShooterCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier shooterSpeed){
        this.shooterSubsystem = shooterSubsystem;
        this.shooterSpeed = shooterSpeed.getAsDouble();
        addRequirements(shooterSubsystem);
    }


    @Override
    public void initialize(){
       shooterSubsystem.resetAllShooterMotorEncoders();
       shooterSubsystem.stopShooterMotors();
    }

    @Override
    public void execute(){
        shooterSubsystem.setShooterSpeed();
    }

    @Override 
    public void end(boolean interrupted){
       shooterSubsystem.stopShooterMotors();
       shooterSubsystem.close();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}