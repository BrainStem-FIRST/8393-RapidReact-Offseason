package frc.robot.subsystems;

import java.lang.reflect.Method;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TransferConstants;

public class TransferSubsystem extends SubsystemBase {

    TransferConstants constants = new TransferConstants();
    public CANSparkMax transfer_motor = new CANSparkMax(TransferConstants.transferMotorPort, MotorType.kBrushless);
    
    public TransferSubsystem(){
    }

    
}
