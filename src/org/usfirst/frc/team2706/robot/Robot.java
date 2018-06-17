
package org.usfirst.frc.team2706.robot;

import org.usfirst.frc.team2706.robot.controls.operatorFeedback.Rumbler;
import org.usfirst.frc.team2706.robot.subsystems.SwerveDriveTrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class controls all of the robot initialization, every tick of the robot, and should be very
 * bare bones to preserve readability and simplicity. Do not change the class name without updating
 * the manifest file, and references to different subsystems should be static. Refer to your local
 * gatekeeper if you have no idea what you are doing :)
 */
public class Robot extends IterativeRobot {

    // The robot's main drive train
    public static SwerveDriveTrain driveTrain;

    // Stores all of the joysticks, and returns them as read only.
    public static OI oi;
    
    // Checks if the robot has entered teleop determine if the match has ended on disable
    private static boolean enteredTeleop;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public void robotInit() {
        Log.setUpLogging();

        Log.i("Robot", "Starting robot code");

        RobotMap.log();

        // Instantiate the robot subsystems
        driveTrain = new SwerveDriveTrain();
    }

    /**
     * This function is called once each time the robot enters Disabled mode. You can use it to
     * reset any subsystem information you want to clear when the robot is disabled.
     */
    public void disabledInit() {
        Log.i("Robot", "Disabled");

        Log.updateTableLog();
        if (enteredTeleop) {
            Log.save();
        }
        // Stop timer on the dashboard
        SmartDashboard.putBoolean("time_running", false);
    }

    public void disabledPeriodic() {
        Scheduler.getInstance().run();
        log();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
     * uncomment the getString code to get the auto name from the text box below the Gyro
     *
     * You can add additional auto modes by adding additional commands to the chooser code above
     * (like the commented example) or additional comparisons to the switch structure below with
     * additional strings & commands.
     */
    public void autonomousInit() {
        // Begin timer.
        SmartDashboard.putBoolean("time_running", true);
        Log.i("Robot", "Entering autonomous mode");
        Log.i("Robot", "Autonomous game specific message: "
                        + DriverStation.getInstance().getGameSpecificMessage());

        driveTrain.reset();

        // Start timer on the dashboard
        SmartDashboard.putBoolean("time_running", true);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        log();
    }

    public void teleopInit() {
        Log.i("Robot", "Entering teleop mode");

        Log.i("Robot", "Teleop game specific message: "
                        + DriverStation.getInstance().getGameSpecificMessage());
        enteredTeleop = true;

        Robot.driveTrain.brakeMode(true);

        // Tell drive team to drive
        new Rumbler(0.2, 0.1, 3, Rumbler.JoystickSelection.DRIVER_JOYSTICK);
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        log();
    }

    @Override
    public void testInit() {
        Log.i("Robot", "Entering test mode");
        initTestMode();
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {}

    private void log() {
        // Don't use unnecessary bandwidth at competition
        if (!DriverStation.getInstance().isFMSAttached()
                        || DriverStation.getInstance().isDisabled()) {
            driveTrain.log();
        }
    }

    public void initTestMode() {
        driveTrain.initTestMode();
    }
}
