package org.usfirst.frc.team2706.robot.controls.swerve;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * A servo and motor pair that allow for a single swerve module to operate
 */
public class SwerveModule {

    /**
     * The motor used to drive the robot forward
     */
    private final SpeedController driveMotor;
    
    /**
     * The servo used to rotate the drive motor
     */
    private final IServo rotationServo;
    
    /**
     * Creates a swerve module
     * 
     * @param driveMotor The motor used to drive the robot forward
     * @param rotationServo The servo used to rotate the drive motor
     */
    public SwerveModule(SpeedController driveMotor, IServo rotationServo) {
        this.driveMotor = driveMotor;
        this.rotationServo = rotationServo;
    }

    /**
     * Gets the drive motor
     * 
     * @return The motor used to drive the robot forward
     */
    public SpeedController getDriveMotor() {
        return driveMotor;
    }
    
    /**
     * Gets the rotation servo
     * 
     * @return The servo used to rotate the drive motor
     */
    public IServo getRotationServo() {
        return rotationServo;
    }
    
    /**
     * Sets a speed and angle for the swerve module
     * 
     * @param speed The speed to drive the drive motor forward
     * @param angle The angle to have the servo go to
     */
    public void set(double speed, double angle) {
        driveMotor.set(speed);
        rotationServo.setDesiredAngle(angle);
    }
}
