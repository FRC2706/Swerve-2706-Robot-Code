package org.usfirst.frc.team2706.robot.subsystems;

import org.usfirst.frc.team2706.robot.Log;
import org.usfirst.frc.team2706.robot.RobotMap;
import org.usfirst.frc.team2706.robot.commands.teleop.ArcadeDriveWithJoystick;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

public class SwerveDriveTrain extends Subsystem {
    
    private Command defaultCommand;
    
    public SwerveDriveTrain() {
        // TODO: Initialize all required motors/sensors
    }
    
    
    public void initTestMode() {    
        // Let's show everything on the LiveWindow
    }
    /**
     * When no other command is running let the operator drive around using the Xbox joystick.
     */
    public void initDefaultCommand() {
        if (defaultCommand == null) {
            getDefaultCommand();
        }
        setDefaultCommand(defaultCommand);

        Log.i("Drive Train Command", defaultCommand);
    }

    public Command getDefaultCommand() {
        if (defaultCommand == null) {
            defaultCommand = new ArcadeDriveWithJoystick();
        }
        return defaultCommand;
    }

    /**
     * The log method puts interesting information to the SmartDashboard.
     */
    public void log() {
        // TODO: Log any motor and sensor values to here
    }

    public void debugLog() {
     // TODO: Log.d() any motor and sensor values to here
    }
    
    public void brakeMode(boolean on) {
     // TODO: Make toggle drive motor brakes
    }
    
    public void reset() {
        // TODO: Reset any sensors here
    }
    
    public void drive(GenericHID joy) {
        double XAxis = RobotMap.INVERT_JOYSTICK_X ? -joy.getRawAxis(4) : joy.getRawAxis(4);
        double YAxis = RobotMap.INVERT_JOYSTICK_Y ? -joy.getRawAxis(5) : joy.getRawAxis(5);
        
        double speed = Math.hypot(XAxis, YAxis);
        
        double heading = Math.toDegrees(Math.atan2(YAxis, XAxis)) - 90;
        
        if(heading < 0) {
            heading += 360;
        }
        
        heading = 360 - heading;
        
        if(Math.abs(XAxis) == 0 && (Math.abs(YAxis) == 0 || YAxis > 0)) {
            heading = 0;
        }
        
        drive(speed, heading);
    }
    
    public void drive(double speed, double heading) {
        // TODO: Pass arguments to SwerveDrive
    }
}
