package org.usfirst.frc.team2706.robot.controls.swerve;

/**
 * A motor that can rotate to an angle given
 */
public interface IServo  {

    /**
     * Sets the angle that the servo should go to
     * 
     * @param degrees The angle the servo should go to in degrees
     */
    void setDesiredAngle(double degrees);
    
    /**
     * Gets the angle that the servo is set to go to
     * 
     * @return The desired angle in degrees
     */
    double getDesiredAngle();
    
    /**
     * The angle that the servo is actually facing
     * 
     * @return The actual angle in degrees
     */
    double getActualAngle();
    
    /**
     * Sets the absolute position of the servo
     * 
     * @param position The absolute position of the servo
     */
    void setDesiredPosition(double position);
    
    /**
     * Gets the desired absolute position of the servo
     * 
     * @return The desired absolute position of the servo
     */
    double getDesiredPosition();
    
    /**
     * Gets the actual absolute position of the servo
     * 
     * @return The actual absolute position of the servo
     */
    double getActualPosition();
    
    /**
     * Stops the servo from running until another set method is called
     */
    void stop();
}
