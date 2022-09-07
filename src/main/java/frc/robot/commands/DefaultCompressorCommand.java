package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CompressorSubsystem;

public class DefaultCompressorCommand extends CommandBase {

    private final CompressorSubsystem compressorSubsystem;
    private final int minPressure;
    private final int maxPressure;

    public DefaultCompressorCommand(CompressorSubsystem compressorSubsystem, int minPressure, int maxPressure) {
        this.compressorSubsystem = compressorSubsystem;
        this.minPressure = minPressure;
        this.maxPressure = maxPressure;
        addRequirements(compressorSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        compressorSubsystem.setPressures(minPressure, maxPressure);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}