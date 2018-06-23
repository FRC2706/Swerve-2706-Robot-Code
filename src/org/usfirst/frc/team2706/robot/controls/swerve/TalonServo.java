package org.usfirst.frc.team2706.robot.controls.swerve;

import org.usfirst.frc.team2706.robot.controls.talon.TalonPID;
import org.usfirst.frc.team2706.robot.controls.talon.TalonSensorGroup;

import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class TalonServo extends SendableBase implements IServo {

    private final TalonSensorGroup talons;
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
        double a1 = getActualAngle();
        if(a1 < 0) {
            a1 += 360;
        }
        
        double a2 = degrees;
        if(a1 < 0) {
            a2 += 360;
        }
        
        setAngleRelative(180 - Math.abs(Math.abs(a1 - a2) - 180));
    }

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
    
    private double normalize(double actual) {
        double modded = actual % 360.0;

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
        // May become problem if servo needs to be disabled
        if(!pid.enabled) pid.enable();
        
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
    
    public TalonSensorGroup getTalons() {
        return talons;
    }
}
