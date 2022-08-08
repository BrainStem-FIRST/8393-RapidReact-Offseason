package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CompressorSubsytem;


public class CompressorCommand extends CommandBase{

    private final CompressorSubsytem compressorSubsytem;
    private final int minPressure;
    private final int maxPressure;

    public CompressorCommand(CompressorSubsytem compressorSubsytem, int minPressure, int maxPressure){
        this.compressorSubsytem = compressorSubsytem;
        this.minPressure = minPressure;
        this.maxPressure = maxPressure; 
        addRequirements(compressorSubsytem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        compressorSubsytem.setPressures(minPressure, maxPressure);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }

}