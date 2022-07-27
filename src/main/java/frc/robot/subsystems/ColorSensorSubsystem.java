package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants.ColorSensorConstants;

public class ColorSensorSubsystem {

    private final I2C.Port ColorSensorI2C = I2C.Port.kOnboard;
    private final ColorSensorV3 transferColorSensor = new ColorSensorV3(ColorSensorI2C);

    private final int currentRValue = transferColorSensor.getRed();
    private final int currentGValue = transferColorSensor.getGreen(); 
    private final int currentBValue = transferColorSensor.getBlue();

    public boolean isCargoRed() {
        if ( (ColorSensorConstants.RED_ALLIANCE_R_VALUE < currentRValue) && 
             (ColorSensorConstants.RED_ALLIANCE_G_VALUE < currentGValue) &&  
             (ColorSensorConstants.RED_ALLIANCE_B_VALUE < currentBValue)
            ) {
            return true;
        } else {
            return false;
         }
    }

    public boolean isCargoBlue() {
        if ( (ColorSensorConstants.BLUE_ALLIANCE_R_VALUE < currentRValue) && 
             (ColorSensorConstants.BLUE_ALLIANCE_G_VALUE < currentGValue) &&  
             (ColorSensorConstants.BLUE_ALLIANCE_B_VALUE < currentBValue)
            ) {
            return true;
        } else {
            return false;
         }
    }

}