package org.usfirst.frc.team2706.robot.controls.swerve;

import edu.wpi.first.wpilibj.Servo;

public class PWMServo extends Servo implements IServo {

    public PWMServo(int channel) {
        super(channel);
    }

    @Override
    public void setDesiredAngle(double degrees) {
        this.setAngle(degrees);
    }

    @Override
    public double getDesiredAngle() {
        return this.getAngle();
    }

    @Override
    public double getActualAngle() {
        // Servo doesn't have current location information
        return this.getAngle();
    }

    @Override
    public void setDesiredPosition(double position) {
        // Servo can't go to absolute value
        this.setAngle(position);
    }

    @Override
    public double getDesiredPosition() {
        // Servo can't go to absolute value
        return this.getAngle();
    }

    @Override
    public double getActualPosition() {
        // Servo can't go to absolute value and doesn't have current location information
        return this.getAngle();
    }
}
