package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.TimerCanceller;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivetrainTestCommand extends CommandBase{
    private DrivetrainSubsystem drivetrainSubsystem;
    private double speed = 0.5;
    private TimerCanceller timerCanceller;

    public DrivetrainTestCommand(DrivetrainSubsystem drivetrainSubsystem, double motorRunTime){
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.timerCanceller = new TimerCanceller(motorRunTime);
    }

    @Override
    public void initialize(){
        drivetrainSubsystem.initialize();
    }

    @Override
    public void execute(){
        drivetrainSubsystem.setBackLeftDriveMotorSpeed(speed);
    }

    @Override
    public void end(boolean isFinished){

    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
