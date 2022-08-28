package frc.robot.subsystems;

import edu.wpi.first.cscore.AxisCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.I2C.Port;

import static frc.robot.Constants.*;
import edu.wpi.first.cameraserver.CameraServer;


public class TestCameraSubsystem {
    AxisCamera camera = AxisCamera.getHttpCameraKindFromInt(11);

}
