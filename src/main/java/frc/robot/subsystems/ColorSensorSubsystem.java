package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants.ColorSensorConstants;

public class ColorSensorSubsystem {

    private final I2C.Port ColorSensorI2C = I2C.Port.kOnboard;
    private final ColorSensorV3 TransferColorSensor = new ColorSensorV3(ColorSensorI2C);

    private final int CurrentRValue = TransferColorSensor.getRed();
    private final int CurrentGValue = TransferColorSensor.getGreen(); 
    private final int CurrentBValue = TransferColorSensor.getBlue();

    public boolean isCargoRed() {
        if ( (ColorSensorConstants.RED_ALLIANCE_R_VALUE < CurrentRValue) && 
             (ColorSensorConstants.RED_ALLIANCE_G_VALUE < CurrentGValue) &&  
             (ColorSensorConstants.RED_ALLIANCE_B_VALUE < CurrentBValue)
            ) {
            return true;
        } else {
            return false;
         }
    }

    public boolean isCargoBlue() {
        if ( (ColorSensorConstants.BLUE_ALLIANCE_R_VALUE < CurrentRValue) && 
             (ColorSensorConstants.BLUE_ALLIANCE_G_VALUE < CurrentGValue) &&  
             (ColorSensorConstants.BLUE_ALLIANCE_B_VALUE < CurrentBValue)
            ) {
            return true;
        } else {
            return false;
         }
    }


}