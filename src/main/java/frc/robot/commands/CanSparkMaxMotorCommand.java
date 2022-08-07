package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CanSparkMaxMotorCommand extends CommandBase {
    private CANSparkMax canSparkMax;

    public CanSparkMaxMotorCommand(double Speed, int motorID){
        canSparkMax = new CANSparkMax(motorID, MotorType.kBrushless);
    }


    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

    }

    @Override 
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }



}
