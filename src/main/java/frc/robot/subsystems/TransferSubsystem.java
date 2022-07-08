package frc.robot.subsystems;

import java.lang.reflect.Method;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TransferSubsystem extends SubsystemBase {

    public CANSparkMax transfer_motor = new CANSparkMax(Constants.TransferConstants.TRANSFER_MOTOR_PORT_ID, MotorType.kBrushless);

    public void motor_on() {}
        
   
    public TransferSubsystem(){
    }

    
}
