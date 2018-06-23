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
        // TODO: Convert to absolute value
        setDesiredPosition(degrees);
    }

    @Override
    public double getDesiredAngle() {
        // TODO: Convert to absolute value
        return getDesiredPosition();
    }

    @Override
    public double getActualAngle() {
        // TODO: Convert to absolute value
        return getActualPosition();
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
