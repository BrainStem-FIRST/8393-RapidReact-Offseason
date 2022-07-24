package frc.robot.subsystems;

import java.lang.reflect.Method;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransferConstants;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;

public class TransferSubsystem extends SubsystemBase implements AutoCloseable {

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 transfer_colorSensor = new ColorSensorV3(i2cPort);
    public CANSparkMax transferMotor = new CANSparkMax(TransferConstants.TRANSFER_MOTOR_PORT_ID,
            MotorType.kBrushless);
    Color ballColor = transfer_colorSensor.getColor();

    public void turnOnTransfer() {
        transferMotor.set(TransferConstants.TRANSFER_MOTOR_SPEED);
    }

    public void turnOff() {
        transferMotor.set(0);
    }

    public void toggleTransfer(boolean enabled) {
        if (transferMotor.get() != 0 && enabled) {
            turnOff();
        } else {
            turnOnTransfer();
        }
    }

    @Override
    public void close() throws Exception{
        transferMotor.close();
    }

}
