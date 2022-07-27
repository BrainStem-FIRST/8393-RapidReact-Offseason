// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase implements AutoCloseable {

        private final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200);
        
        /*
         * private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
         * // Front left
         * new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
         * DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
         * // Front right
         * new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
         * -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
         * // Back left
         * new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
         * DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
         * // Back right
         * new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
         * -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));
         */
        // private final SwerveModule frontLeftModule;
        // private final SwerveModule frontRightModule;
        // private final SwerveModule backLeftModule;
        // private final SwerveModule backRightModule;


        // ALL DRIVETRAIN FUNCTIONS:
        private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(AutoConstants.autoDriveKinematics,
                        new Rotation2d(0));

        // resets heading
        public void zeroHeading() {
                navx.reset();
        }

        // returns heading
        public double getHeading() {
                return Math.IEEEremainder(navx.getAngle(), 360);
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
                navx.zeroYaw();
        }

        // sets motor powers to zero
        public void stopModules() {
                frontLeftModule.stop();
                backLeftModule.stop();
                frontRightModule.stop();
                backRightModule.stop();
        }

        public void setModuleStates(SwerveModuleState[] desiredStates) {
                SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND);
                frontLeftModule.setDesiredState(desiredStates[0]);
                frontRightModule.setDesiredState(desiredStates[1]);
                backLeftModule.setDesiredState(desiredStates[2]);
                backRightModule.setDesiredState(desiredStates[3]);
        }

        public void setModuleStates(ChassisSpeeds chassisSpeeds){
                SwerveModuleState[] desiredStates = Constants.DrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND);
                frontLeftModule.setDesiredState(desiredStates[0]);
                frontRightModule.setDesiredState(desiredStates[1]);
                backLeftModule.setDesiredState(desiredStates[2]);
                backRightModule.setDesiredState(desiredStates[3]);
        }

        // sets voltage as zero for drive motors and keeps turning motors in place
        /*
         * public void stopModules() {
         * backLeftModule.set(0, backLeftModule.getSteerAngle());
         * frontLeftModule.set(0, frontLeftModule.getSteerAngle());
         * backRightModule.set(0, backRightModule.getSteerAngle());
         * frontRightModule.set(0, frontRightModule.getSteerAngle());
         * }
         */

        public Rotation2d getGyroscopeRotation() {
                // if (m_navx.isMagnetometerCalibrated()) {
                // We will only get valid fused headings if the magnetometer is calibrated
                // return Rotation2d.fromDegrees(m_navx.getFusedHeading());
                // }

                // We have to invert the angle of the NavX so that rotating the robot
                // counter-clockwise makes the angle increase.
                return Rotation2d.fromDegrees(navx.getYaw()); // IF DOESN'T WORK DO 180 - navx.getYaw()
        }

        //private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0,
                //        0.0);

        public DrivetrainSubsystem() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

                new Thread(() -> {
                        try {
                                Thread.sleep(1000);
                                zeroHeading();
                        } catch (Exception e) {
                        }
                }).start();
        }

        private final SwerveModuleSubsystem frontLeftModule = new SwerveModuleSubsystem(
                        DrivetrainConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                        DrivetrainConstants.FRONT_LEFT_MODULE_TURNING_MOTOR,
                        DrivetrainConstants.FRONT_LEFT_TURNING_ABSOLUTE_ENCODER_REVERSED,
                        DrivetrainConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED,
                        DrivetrainConstants.FRONT_LEFT_MODULE_TURNING_ABSOLUTE_ENCODER,
                        DrivetrainConstants.FRONT_LEFT_MODULE_TURNING_OFFSET,
                        DrivetrainConstants.FRONT_LEFT_TURNING_ABSOLUTE_ENCODER_REVERSED);

        private final SwerveModuleSubsystem frontRightModule = new SwerveModuleSubsystem(
                        DrivetrainConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                        DrivetrainConstants.FRONT_RIGHT_MODULE_TURNING_MOTOR,
                        DrivetrainConstants.FRONT_RIGHT_TURNING_ABSOLUTE_ENCODER_REVERSED,
                        DrivetrainConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED,
                        DrivetrainConstants.FRONT_RIGHT_MODULE_TURNING_ABSOLUTE_ENCODER,
                        DrivetrainConstants.FRONT_RIGHT_MODULE_TURNING_OFFSET,
                        DrivetrainConstants.FRONT_RIGHT_TURNING_ABSOLUTE_ENCODER_REVERSED);

        private final SwerveModuleSubsystem backLeftModule = new SwerveModuleSubsystem(
                        DrivetrainConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                        DrivetrainConstants.BACK_LEFT_MODULE_TURNING_MOTOR,
                        DrivetrainConstants.BACK_LEFT_TURNING_ABSOLUTE_ENCODER_REVERSED,
                        DrivetrainConstants.BACK_LEFT_DRIVE_ENCODER_REVERSED,
                        DrivetrainConstants.BACK_LEFT_MODULE_TURNING_ABSOLUTE_ENCODER,
                        DrivetrainConstants.BACK_LEFT_MODULE_TURNING_OFFSET,
                        DrivetrainConstants.BACK_LEFT_TURNING_ABSOLUTE_ENCODER_REVERSED);

        private final SwerveModuleSubsystem backRightModule = new SwerveModuleSubsystem(
                        DrivetrainConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                        DrivetrainConstants.BACK_RIGHT_MODULE_TURNING_MOTOR,
                        DrivetrainConstants.BACK_RIGHT_TURNING_ABSOLUTE_ENCODER_REVERSED,
                        DrivetrainConstants.BACK_RIGHT_DRIVE_ENCODER_REVERSED,
                        DrivetrainConstants.BACK_RIGHT_MODULE_TURNING_ABSOLUTE_ENCODER,
                        DrivetrainConstants.BACK_RIGHT_MODULE_TURNING_OFFSET,
                        DrivetrainConstants.BACK_RIGHT_TURNING_ABSOLUTE_ENCODER_REVERSED);

        /*
         * frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
         * // This parameter is optional, but will allow you to see the current state of
         * // the module on the dashboard.
         * tab.getLayout("Front Left Module", BuiltInLayouts.kList)
         * .withSize(2, 4)
         * .withPosition(0, 0),
         * // This can either be STANDARD or FAST depending on your gear configuration
         * Mk4SwerveModuleHelper.GearRatio.L4,
         * // This is the ID of the drive motor
         * DrivetrainConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
         * // This is the ID of the steer motor
         * DrivetrainConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
         * // This is the ID of the steer encoder
         * DrivetrainConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
         * // This is how much the steer encoder is offset from true zero (In our case,
         * // zero is facing straight forward)
         * DrivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET);
         * 
         * 
         * 
         * // We will do the same for the other modules
         * frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
         * tab.getLayout("Front Right Module", BuiltInLayouts.kList)
         * .withSize(2, 4)
         * .withPosition(2, 0),
         * Mk4SwerveModuleHelper.GearRatio.L4,
         * DrivetrainConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
         * DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
         * DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
         * DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);
         * 
         * backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
         * tab.getLayout("Back Left Module", BuiltInLayouts.kList)
         * .withSize(2, 4)
         * .withPosition(4, 0),
         * Mk4SwerveModuleHelper.GearRatio.L4,
         * DrivetrainConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
         * DrivetrainConstants.BACK_LEFT_MODULE_STEER_MOTOR,
         * DrivetrainConstants.BACK_LEFT_MODULE_STEER_ENCODER,
         * DrivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET);
         * 
         * backRightModule = Mk4SwerveModuleHelper.createFalcon500(
         * tab.getLayout("Back Right Module", BuiltInLayouts.kList)
         * .withSize(2, 4)
         * .withPosition(6, 0),
         * Mk4SwerveModuleHelper.GearRatio.L4,
         * DrivetrainConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
         * DrivetrainConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
         * DrivetrainConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
         * DrivetrainConstants.BACK_RIGHT_MODULE_STEER_OFFSET);
         * }
         */

        // public void drive(ChassisSpeeds chassisSpeeds) {
        // m_chassisSpeeds = chassisSpeeds;
        // }

        /*
         * public void setModuleStates(SwerveModuleState[] desiredStates) {
         * SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
         * ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND);
         * frontLeftModule.set(
         * desiredStates[0].speedMetersPerSecond
         * / ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND *
         * ConstraintsConstants.MAX_ROBOT_VOLTAGE,
         * desiredStates[0].angle.getRadians());
         * frontRightModule.set(
         * desiredStates[1].speedMetersPerSecond
         * / ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND *
         * ConstraintsConstants.MAX_ROBOT_VOLTAGE,
         * desiredStates[1].angle.getRadians());
         * backLeftModule.set(
         * desiredStates[2].speedMetersPerSecond
         * / ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND *
         * ConstraintsConstants.MAX_ROBOT_VOLTAGE,
         * desiredStates[2].angle.getRadians());
         * backRightModule.set(
         * desiredStates[3].speedMetersPerSecond
         * / ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND *
         * ConstraintsConstants.MAX_ROBOT_VOLTAGE,
         * desiredStates[3].angle.getRadians());
         * }
         * 
         * public void setModuleStates(ChassisSpeeds chassisSpeeds) {
         * SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
         * SwerveDriveKinematics.desaturateWheelSpeeds(states,
         * ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND);
         * frontLeftModule.set(
         * states[0].speedMetersPerSecond
         * / ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND *
         * ConstraintsConstants.MAX_ROBOT_VOLTAGE,
         * states[0].angle.getRadians());
         * frontRightModule.set(
         * states[1].speedMetersPerSecond
         * / ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND *
         * ConstraintsConstants.MAX_ROBOT_VOLTAGE,
         * states[1].angle.getRadians());
         * backLeftModule.set(
         * states[2].speedMetersPerSecond
         * / ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND *
         * ConstraintsConstants.MAX_ROBOT_VOLTAGE,
         * states[2].angle.getRadians());
         * backRightModule.set(
         * states[3].speedMetersPerSecond
         * / ConstraintsConstants.MAX_VELOCITY_METERS_PER_SECOND *
         * ConstraintsConstants.MAX_ROBOT_VOLTAGE,
         * states[3].angle.getRadians());
         * }
         * 
         * public void stopAllMotors(){
         * setModuleStates(new ChassisSpeeds(0, 0, 0));
         * }
         */

        @Override
        public void periodic() {
                odometer.update(getRotation2d(), frontLeftModule.getState(), frontRightModule.getState(),
                                backLeftModule.getState(), backRightModule.getState());
                // SwerveModuleState[] states =
                // m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                // setModuleStates(states);
        }

        @Override
        public void close() {
                
        }

}