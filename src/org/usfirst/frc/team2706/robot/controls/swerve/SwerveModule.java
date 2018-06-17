package org.usfirst.frc.team2706.robot.controls.swerve;

import edu.wpi.first.wpilibj.SpeedController;

public class SwerveModule {

    private final SpeedController driveMotor;
    private final IServo rotationServo;
    
    public SwerveModule(SpeedController driveMotor, IServo rotationServo) {
        this.driveMotor = driveMotor;
        this.rotationServo = rotationServo;
    }

    public SpeedController getDriveMotor() {
        return driveMotor;
    }
    
    public IServo getRotationServo() {
        return rotationServo;
    }
}
