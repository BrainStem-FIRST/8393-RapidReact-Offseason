package com.swervedrivespecialties.swervelib;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    double getDriveVelocity();
    
    double getSteerAngle();

    SwerveModuleState getState();

    void setDesiredState();

    void set(double driveVoltage, double steerAngle);
    void stop();
}
