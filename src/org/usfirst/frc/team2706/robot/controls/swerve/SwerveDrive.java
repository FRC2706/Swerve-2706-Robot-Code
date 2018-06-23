package org.usfirst.frc.team2706.robot.controls.swerve;

import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;

import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class SwerveDrive extends RobotDriveBase {
    
    private final SwerveModule frontLeftSwerve, backLeftSwerve, frontRightSwerve, backRightSwerve;
    
    // Convenient for applying same operation to each module
    private final Collection<SwerveModule> modules;

    public SwerveDrive(SwerveModule frontLeftSwerve, SwerveModule backLeftSwerve,
                    SwerveModule frontRightSwerve, SwerveModule backRightSwerve) {
        this.frontLeftSwerve = frontLeftSwerve;
        this.backLeftSwerve = backLeftSwerve;
        this.frontRightSwerve = frontRightSwerve;
        this.backRightSwerve = backRightSwerve;
        
        modules = Collections.unmodifiableCollection(
                        Arrays.asList(backLeftSwerve, backRightSwerve, frontLeftSwerve, frontRightSwerve));
    }
    
    public void swerveDrive(double forward, double strafe, double rotate) {
        swerveDrive(forward, strafe, rotate, true);
    }
    
    public void swerveDrive(double forward, double strafe, double rotate, boolean squaredInputs) {
        
    }
    
    @Override
    public void stopMotor() {
        modules.forEach((swerve) -> swerve.getDriveMotor().stopMotor());
        
        m_safetyHelper.feed();
    }

    @Override
    public String getDescription() {
        return "SwerveDrive";
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        
        builder.addDoubleProperty("Front Left Rotation", frontLeftSwerve.getRotationServo()::getDesiredAngle,
                        frontLeftSwerve.getRotationServo()::setDesiredAngle);
        builder.addDoubleProperty("Front Left Speed", frontLeftSwerve.getDriveMotor()::get,
                        frontLeftSwerve.getDriveMotor()::set);
        
        builder.addDoubleProperty("Back Left Rotation", backLeftSwerve.getRotationServo()::getDesiredAngle,
                        backLeftSwerve.getRotationServo()::setDesiredAngle);
        builder.addDoubleProperty("Back Left Speed", backLeftSwerve.getDriveMotor()::get,
                        backLeftSwerve.getDriveMotor()::set);
        
        builder.addDoubleProperty("Front Right Rotation", frontRightSwerve.getRotationServo()::getDesiredAngle,
                        frontRightSwerve.getRotationServo()::setDesiredAngle);
        builder.addDoubleProperty("Front Right Speed", frontRightSwerve.getDriveMotor()::get,
                        frontRightSwerve.getDriveMotor()::set);
        
        builder.addDoubleProperty("Back Right Rotation", backRightSwerve.getRotationServo()::getDesiredAngle,
                        backRightSwerve.getRotationServo()::setDesiredAngle);
        builder.addDoubleProperty("Back Right Speed", backRightSwerve.getDriveMotor()::get,
                        backRightSwerve.getDriveMotor()::set);
    }
}
