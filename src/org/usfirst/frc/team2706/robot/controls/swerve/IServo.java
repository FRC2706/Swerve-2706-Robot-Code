package org.usfirst.frc.team2706.robot.controls.swerve;

public interface IServo  {

    void setDesiredAngle(double degrees);
    double getDesiredAngle();
    
    double getActualAngle();
    
    void setDesiredPosition(double position);
    double getDesiredPosition();
    
    double getActualPosition();
    
    void stop();
}
