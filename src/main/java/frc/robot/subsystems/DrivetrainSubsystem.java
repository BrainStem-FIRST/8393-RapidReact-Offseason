// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
//import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
//import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase implements AutoCloseable {
        /**
         * The maximum voltage that will be delivered to the drive motors.
         * <p>
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.
         */
        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // By default this value is setup for a Mk3 standard module using Falcon500s to
        // drive.
        // An example of this constant for a Mk4 L1 module with NEOs to drive is:
        // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L1.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI
        /**
         * The maximum velocity of the robot in meters per second.
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight
         * line.
         */
        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.

        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Front right
                        new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back left
                        new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back right
                        new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

        private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

        // These are our modules. We initialize them in the constructor.
        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;

        // ALL DRIVETRAIN FUNCTIONS:
        private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(AutoConstants.kDriveKinematics,
                        new Rotation2d(0));

        // resets heading
        public void zeroHeading() {
                m_navx.reset();
        }

        // returns heading
        public double getHeading() {
                return Math.IEEEremainder(m_navx.getAngle(), 360);
        }

        // returns heading
        public Rotation2d getRotation2d() {
                return Rotation2d.fromDegrees(getHeading());
        }

        // returns location and heading
        public Pose2d getPose() {
                return odometer.getPoseMeters();
        }

        // resets odometry
        public void resetOdometry(Pose2d pose) {
                odometer.resetPosition(pose, getRotation2d());
        }

        // sets gyro to zero
        public void zeroGyroscope() {
                m_navx.zeroYaw();
        }

        // sets voltage as zero for drive motors and keeps turning motors in place
        public void stopModules() {
                m_backLeftModule.set(0, m_backLeftModule.getSteerAngle());
                m_frontLeftModule.set(0, m_frontLeftModule.getSteerAngle());
                m_backRightModule.set(0, m_backRightModule.getSteerAngle());
                m_frontRightModule.set(0, m_frontRightModule.getSteerAngle());
        }

        public Rotation2d getGyroscopeRotation() {
                // if (m_navx.isMagnetometerCalibrated()) {
                // We will only get valid fused headings if the magnetometer is calibrated
                // return Rotation2d.fromDegrees(m_navx.getFusedHeading());
                // }

                // We have to invert the angle of the NavX so that rotating the robot
                // counter-clockwise makes the angle increase.
                return Rotation2d.fromDegrees(180 - m_navx.getYaw());
        }

        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0,
                        0.0);

        public DrivetrainSubsystem() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

                // There are 4 methods you can call to create your swerve modules.
                // The method you use depends on what motors you are using.
                //
                // Mk3SwerveModuleHelper.createFalcon500(...)
                // Your module has two Falcon 500s on it. One for steering and one for driving.
                //
                // Mk3SwerveModuleHelper.createNeo(...)
                // Your module has two NEOs on it. One for steering and one for driving.
                //
                // Mk3SwerveModuleHelper.createFalcon500Neo(...)
                // Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving
                // and the NEO is for steering.
                //
                // Mk3SwerveModuleHelper.createNeoFalcon500(...)
                // Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the
                // Falcon 500 is for steering.
                //
                // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper
                // class.

                // By default we will use Falcon 500s in standard configuration. But if you use
                // a different configuration or motors
                // you MUST change it. If you do not, your code will crash on startup.

                // Mihir added code here too
                new Thread(() -> {
                        try {
                                Thread.sleep(1000);
                                zeroHeading();
                        } catch (Exception e) {
                        }
                }).start();

                m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                                // This parameter is optional, but will allow you to see the current state of
                                // the module on the dashboard.
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                // This can either be STANDARD or FAST depending on your gear configuration
                                Mk4SwerveModuleHelper.GearRatio.L4,
                                // This is the ID of the drive motor
                                DrivetrainConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                // This is the ID of the steer motor
                                DrivetrainConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
                                // This is the ID of the steer encoder
                                DrivetrainConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
                                // This is how much the steer encoder is offset from true zero (In our case,
                                // zero is facing straight forward)
                                DrivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET);

                // We will do the same for the other modules
                m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                Mk4SwerveModuleHelper.GearRatio.L4,
                                DrivetrainConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                                DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                                DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);

                m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                Mk4SwerveModuleHelper.GearRatio.L4,
                                DrivetrainConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                                DrivetrainConstants.BACK_LEFT_MODULE_STEER_MOTOR,
                                DrivetrainConstants.BACK_LEFT_MODULE_STEER_ENCODER,
                                DrivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET);

                m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                Mk4SwerveModuleHelper.GearRatio.L4,
                                DrivetrainConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                DrivetrainConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
                                DrivetrainConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
                                DrivetrainConstants.BACK_RIGHT_MODULE_STEER_OFFSET);
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;
        }

        public void setModuleStates(SwerveModuleState[] desiredStates) {
                SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND);
                m_frontLeftModule.set(
                                desiredStates[0].speedMetersPerSecond / ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND *
                                 ConstraintsConstants.MAX_ROBOT_VOLTAGE,
                                desiredStates[0].angle.getRadians());
                m_frontRightModule.set(
                                desiredStates[1].speedMetersPerSecond / ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND * 
                                ConstraintsConstants.MAX_ROBOT_VOLTAGE,
                                desiredStates[1].angle.getRadians());
                m_backLeftModule.set(
                                desiredStates[2].speedMetersPerSecond / ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND * 
                                ConstraintsConstants.MAX_ROBOT_VOLTAGE,
                                desiredStates[2].angle.getRadians());
                m_backRightModule.set(
                                desiredStates[3].speedMetersPerSecond / ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND * 
                                ConstraintsConstants.MAX_ROBOT_VOLTAGE,
                                desiredStates[3].angle.getRadians());
        }

        @Override
        public void periodic() {
                odometer.update(getRotation2d(), m_frontLeftModule.getState(), m_frontRightModule.getState(),
                                m_backLeftModule.getState(), m_backRightModule.getState());
                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                setModuleStates(states);
        }
        @Override
    public void close(){}

}