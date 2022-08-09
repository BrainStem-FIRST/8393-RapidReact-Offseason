package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstraintsConstants;
import frc.robot.Constants.TransferConstants;

public class TransferSubsystem extends SubsystemBase implements AutoCloseable {

    

    public CANSparkMax transferMotor = new CANSparkMax(TransferConstants.TRANSFER_MOTOR_PORT_ID,
            MotorType.kBrushless);

    private RelativeEncoder transferMotorEncoder = transferMotor.getEncoder();

    public void turnOn(double speed) {
        
        transferMotor.set(speed);
    }

    public void turnOff() {
        transferMotor.set(0);
    }

    public void initializeTransfer() {
        transferMotorEncoder.setPosition(0);
        turnOff();
    }

    public void executeTransfer(boolean turnOnTransfer) {
        if (turnOnTransfer) {
            turnOn(-1);
        } else {
            turnOff();
        }

       
       


    }

    public void endTransfer(){
        initializeTransfer();
    }

    @Override
    public void close() throws Exception {
        transferMotor.close();
    }

}
