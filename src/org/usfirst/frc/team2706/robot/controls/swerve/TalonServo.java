package org.usfirst.frc.team2706.robot.controls.swerve;

import org.usfirst.frc.team2706.robot.controls.talon.TalonPID;
import org.usfirst.frc.team2706.robot.controls.talon.TalonSensorGroup;

import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * A servo that uses the TalonSRX API to control its position
 */
public class TalonServo extends SendableBase implements IServo {

    /**
     * The Talon(s) used to rotate
     */
    private final TalonSensorGroup talons;
    
    /**
     * The PID that controls the position loop
     */
    private final TalonPID pid;
    
    /**
     * Creates TalonServo that assumes that one encoder distance unit represents one degree of rotation
     * 
     * @param talons The motor controller that will control rotation
     */
    public TalonServo(TalonSensorGroup talons) {
        this.talons = talons;
        this.pid = new TalonPID(talons);
    }
    
    @Override
    public void setDesiredAngle(double degrees) {
        // Convert to 0 to 360 angle
        double a1 = getActualAngle();
        if(a1 < 0) {
            a1 += 360;
        }
        
        // Convert to 0 to 360 angle
        double a2 = normalize(degrees);
        if(a1 < 0) {
            a2 += 360;
        }
        
        // Find difference in angles from
        // https://gamedev.stackexchange.com/questions/4467/comparing-angles-and-working-out-the-difference
        setAngleRelative(180 - Math.abs(Math.abs(a1 - a2) - 180));
    }

    /**
     * Sets the angle to change by
     * 
     * @param angleChange The angle to change by
     */
    public void setAngleRelative(double angleChange) {
        setDesiredPosition(getActualPosition() + angleChange);
    }
    
    @Override
    public double getDesiredAngle() {
        return normalize(getDesiredPosition());
    }

    @Override
    public double getActualAngle() {
        return normalize(getActualPosition());
    }
    
    /**
     * Converts a position to an angle assuming that 360 distance is 1 full rotation
     * 
     * @param actual The angle to normalize
     * @return The normalize angle
     */
    private double normalize(double actual) {
        // Remove full revolutions
        double modded = actual % 360.0;

        // If the angle is above 180, make it less and invert the sign
        double normalized;
        if(modded > 180) {
            normalized = modded - 360.0;
        }
        else if(modded < -180) {
            normalized = modded + 360.0;
        }
        else {
            normalized = modded;
        }

        
        return normalized;
    }

    @Override
    public void setDesiredPosition(double position) {
        // Enable the servo again
        if(!pid.isEnabled()) pid.enable();
        
        pid.setSetpoint(position);
    }

    @Override
    public double getDesiredPosition() {
        return pid.getSetpoint();
    }

    @Override
    public double getActualPosition() {
        return talons.getTalonEncoder().getDistance();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("TalonServo");
        builder.addDoubleProperty("Normalized", this::getDesiredAngle, this::setDesiredAngle);
        builder.addDoubleProperty("Absolute", this::getDesiredPosition, this::setDesiredPosition);
    }
    
    /**
     * Gets the TalonSensorGroup that is doing the servoing
     * 
     * @return The TalonSensorGroup
     */
    public TalonSensorGroup getTalons() {
        return talons;
    }

    @Override
    public void stop() {
        pid.disable();
    }
}
