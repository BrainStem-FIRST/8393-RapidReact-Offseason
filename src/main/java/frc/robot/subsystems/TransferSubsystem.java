package frc.robot.subsystems;

import java.lang.reflect.Method;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransferConstants;;

public class TransferSubsystem extends SubsystemBase implements AutoCloseable {

    public CANSparkMax transferMotor = new CANSparkMax(TransferConstants.TRANSFER_MOTOR_PORT_ID,
            MotorType.kBrushless);

    public void turnOnTransfer() {
        transferMotor.set(TransferConstants.TRANSFER_MOTOR_SPEED);
    }
    public void turnOff(){
        transferMotor.set(0);
    }
    public void toggleTransfer(boolean enabled){
        if(transferMotor.get() != 0 && enabled){
            turnOff();
        }else{
            turnOn();
        }
    }

    @Override
    public void close() {
        transferMotor.close();
    }

}
