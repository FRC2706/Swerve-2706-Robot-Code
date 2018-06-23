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

    private final double wheelBase, trackWidth;

    private final double radius;

    public SwerveDrive(SwerveModule frontLeftSwerve, SwerveModule backLeftSwerve,
                    SwerveModule frontRightSwerve, SwerveModule backRightSwerve, double wheelBase,
                    double trackWidth) {
        this.frontLeftSwerve = frontLeftSwerve;
        this.backLeftSwerve = backLeftSwerve;
        this.frontRightSwerve = frontRightSwerve;
        this.backRightSwerve = backRightSwerve;

        modules = Collections.unmodifiableCollection(Arrays.asList(backLeftSwerve, backRightSwerve,
                        frontLeftSwerve, frontRightSwerve));

        this.wheelBase = wheelBase;
        this.trackWidth = trackWidth;

        this.radius = Math.hypot(wheelBase, trackWidth);
    }

    public void swerveDrive(double forward, double strafe, double rotate) {
        swerveDrive(forward, strafe, rotate, true);
    }

    public void swerveDrive(double forward, double strafe, double rotate, boolean squaredInputs) {
        if (squaredInputs) {
            forward *= forward;
            strafe *= strafe;
            rotate *= rotate;
        }

        // Original algorithm can be found here:
        // http://www.chiefdelphi.com/media/papers/download/3028
        double a = strafe - rotate * (wheelBase / radius);
        double b = strafe + rotate * (wheelBase / radius);
        double c = forward - rotate * (trackWidth / radius);
        double d = forward + rotate * (trackWidth / radius);

        double frontRightSpeed = Math.hypot(b, c);
        double frontRightAngle = Math.toDegrees(Math.atan2(b, c));

        double frontLeftSpeed = Math.hypot(b, d);
        double frontLeftAngle = Math.toDegrees(Math.atan2(b, d));

        double backRightSpeed = Math.hypot(a, d);
        double backRightAngle = Math.toDegrees(Math.atan2(a, d));

        double backLeftSpeed = Math.hypot(a, c);
        double backLeftAngle = Math.toDegrees(Math.atan2(a, c));

        double max = frontRightSpeed;
        if (frontLeftSpeed > max)
            max = frontLeftSpeed;
        if (backRightSpeed > max)
            max = backRightSpeed;
        if (backLeftSpeed > max)
            max = backLeftSpeed;

        if (max > 1) {
            frontRightSpeed /= max;
            frontLeftSpeed /= max;
            backRightSpeed /= max;
            backLeftSpeed /= max;
        }

        frontRightSwerve.set(frontRightSpeed, frontRightAngle);
        frontLeftSwerve.set(frontLeftSpeed, frontLeftAngle);
        backRightSwerve.set(backRightSpeed, backRightAngle);
        backLeftSwerve.set(backLeftSpeed, backLeftAngle);
    }

    @Override
    public void stopMotor() {
        modules.forEach((swerve) -> swerve.getDriveMotor().stopMotor());

        m_safetyHelper.feed();
    }
    

    public void stopSwerve() {
        stopMotor();
        
        modules.forEach((swerve) -> swerve.getRotationServo().stop());
    }

    @Override
    public String getDescription() {
        return "SwerveDrive";
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Rotation",
                        frontLeftSwerve.getRotationServo()::getDesiredAngle,
                        frontLeftSwerve.getRotationServo()::setDesiredAngle);
        builder.addDoubleProperty("Front Left Speed", frontLeftSwerve.getDriveMotor()::get,
                        frontLeftSwerve.getDriveMotor()::set);

        builder.addDoubleProperty("Back Left Rotation",
                        backLeftSwerve.getRotationServo()::getDesiredAngle,
                        backLeftSwerve.getRotationServo()::setDesiredAngle);
        builder.addDoubleProperty("Back Left Speed", backLeftSwerve.getDriveMotor()::get,
                        backLeftSwerve.getDriveMotor()::set);

        builder.addDoubleProperty("Front Right Rotation",
                        frontRightSwerve.getRotationServo()::getDesiredAngle,
                        frontRightSwerve.getRotationServo()::setDesiredAngle);
        builder.addDoubleProperty("Front Right Speed", frontRightSwerve.getDriveMotor()::get,
                        frontRightSwerve.getDriveMotor()::set);

        builder.addDoubleProperty("Back Right Rotation",
                        backRightSwerve.getRotationServo()::getDesiredAngle,
                        backRightSwerve.getRotationServo()::setDesiredAngle);
        builder.addDoubleProperty("Back Right Speed", backRightSwerve.getDriveMotor()::get,
                        backRightSwerve.getDriveMotor()::set);
    }
}
