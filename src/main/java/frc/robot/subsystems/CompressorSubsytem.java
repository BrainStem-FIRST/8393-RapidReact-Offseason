package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PnuematicsConstants;

public class CompressorSubsytem extends SubsystemBase{

    Compressor compressor = new Compressor(PnuematicsConstants.PNEUMATICS_PORT, PneumaticsModuleType.REVPH);

    double pressure = compressor.getPressure();

    public CompressorSubsytem(){
        ShuffleboardTab tab = Shuffleboard.getTab("Compressor");
        tab.add("Pressure Sensor Reading", pressure);
        
    }


    public void setPressures(int minPressure, int maxPressure){
        compressor.enableAnalog(minPressure, maxPressure);

    }

    public double getPressure(){
         return pressure;
    }

    
    
}
