package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {

    private ShooterSubsystem shooterSubsystem;
    private double shooterSpeed;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, double shooterSpeed) {
        this.shooterSubsystem = shooterSubsystem;
        this.shooterSpeed = shooterSpeed;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.initShooter();
    }

    @Override
    public void execute() {
        shooterSubsystem.executeShooter(shooterSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.endShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
