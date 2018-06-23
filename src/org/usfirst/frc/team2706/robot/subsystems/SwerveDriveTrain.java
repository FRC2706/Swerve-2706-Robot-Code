package org.usfirst.frc.team2706.robot.subsystems;

import org.usfirst.frc.team2706.robot.Log;
import org.usfirst.frc.team2706.robot.RobotMap;
import org.usfirst.frc.team2706.robot.commands.teleop.ArcadeDriveWithJoystick;
import org.usfirst.frc.team2706.robot.controls.swerve.SwerveDrive;
import org.usfirst.frc.team2706.robot.controls.swerve.SwerveModule;
import org.usfirst.frc.team2706.robot.controls.swerve.TalonServo;
import org.usfirst.frc.team2706.robot.controls.talon.EWPI_TalonSRX;
import org.usfirst.frc.team2706.robot.controls.talon.IWPI_TalonSRX;
import org.usfirst.frc.team2706.robot.controls.talon.MockWPI_TalonSRX;
import org.usfirst.frc.team2706.robot.controls.talon.TalonEncoder;
import org.usfirst.frc.team2706.robot.controls.talon.TalonSensorGroup;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Runs the driving system of a robot with a swerve drive base
 */
public class SwerveDriveTrain extends Subsystem {
    
    /**
     * Each of the modules used to drive the robot
     */
    private final SwerveModule frontLeftSwerve, backLeftSwerve, frontRightSwerve, backRightSwerve;
    
    /**
     * Each of the rotation encoders on the motors used to rotate the motors
     */
    private final TalonEncoder frontLeftEncoder, backLeftEncoder, frontRightEncoder, backRightEncoder;

    /**
     * The drive base used to control the swerve modules in teleop
     */
    private SwerveDrive drive;

    /**
     * The default command to run in teleop
     */
    private Command defaultCommand;

    /**
     * Creates a swerve drive and all of the required motors and sensors
     */
    public SwerveDriveTrain() {
        super();

        // Initialize the swerve modules and encoders
        frontLeftSwerve = makeSwerveModule(RobotMap.MOTOR_FRONT_LEFT_ROTATION, RobotMap.MOTOR_FRONT_LEFT_DRIVE);
        frontLeftEncoder = ((TalonServo)frontLeftSwerve.getRotationServo()).getTalons().getTalonEncoder();
        
        backLeftSwerve = makeSwerveModule(RobotMap.MOTOR_BACK_LEFT_ROTATION, RobotMap.MOTOR_BACK_LEFT_DRIVE);
        backLeftEncoder = ((TalonServo)backLeftSwerve.getRotationServo()).getTalons().getTalonEncoder();
        
        frontRightSwerve = makeSwerveModule(RobotMap.MOTOR_FRONT_RIGHT_ROTATION, RobotMap.MOTOR_FRONT_RIGHT_DRIVE);
        frontRightEncoder = ((TalonServo)frontRightSwerve.getRotationServo()).getTalons().getTalonEncoder();
        
        backRightSwerve = makeSwerveModule(RobotMap.MOTOR_BACK_RIGHT_ROTATION, RobotMap.MOTOR_BACK_RIGHT_DRIVE);
        backRightEncoder = ((TalonServo)backRightSwerve.getRotationServo()).getTalons().getTalonEncoder();

        drive = new SwerveDrive(frontLeftSwerve, backLeftSwerve, frontRightSwerve, backRightSwerve,
                        RobotMap.WHEELBASE, RobotMap.TRACK_WIDTH);
    }
    
    /**
     * Creates a swerve module using Talon servoes and drive motors
     * 
     * @param rotate The ID of the rotational Talon (or -1 to use mock Talon)
     * @param forward The ID of the drive Talon (or -1 to use mock Talon)
     * @return The swerve drive
     */
    private SwerveModule makeSwerveModule(int rotate, int forward) {
        // Create the rotate Talon, unless it should be a mock Talon
        IWPI_TalonSRX rotateMotor;
        if(rotate == -1) {
            rotateMotor = new MockWPI_TalonSRX();
        }
        else {
            rotateMotor = new EWPI_TalonSRX(rotate);
        }
        
        // Create a servo with the rotate Talon
        TalonServo servo = new TalonServo(new TalonSensorGroup(rotateMotor, new TalonEncoder(rotateMotor)));
        
        // Create the drive Talon, unless it should be a mock Talon
        IWPI_TalonSRX forwardMotor;
        if(forward == -1) {
            forwardMotor = new MockWPI_TalonSRX();
        }
        else {
            forwardMotor = new EWPI_TalonSRX(forward);
        }
        
        // Create the swerve module
        return new SwerveModule(forwardMotor, servo);
    }
    public void initTestMode() {
        // Let's show everything on the LiveWindow
    }

    /**
     * When no other command is running let the operator drive around using the Xbox joystick.
     */
    @Override
    public void initDefaultCommand() {
        if (defaultCommand == null) {
            getDefaultCommand();
        }
        setDefaultCommand(defaultCommand);

        Log.i("Drive Train Command", defaultCommand);
    }
    
    @Override
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
        SmartDashboard.putNumber("Front Left Distance", frontLeftEncoder.getDistance());
        SmartDashboard.putNumber("Back Left Distance", backLeftEncoder.getDistance());
        SmartDashboard.putNumber("Front Right Distance", frontRightEncoder.getDistance());
        SmartDashboard.putNumber("Back Right Distance", backRightEncoder.getDistance());
        
        SmartDashboard.putNumber("Front Left Speed", frontLeftEncoder.getRate());
        SmartDashboard.putNumber("Back Left Speed", backLeftEncoder.getRate());
        SmartDashboard.putNumber("Front Right Speed", frontRightEncoder.getRate());
        SmartDashboard.putNumber("Back Right Speed", backRightEncoder.getRate());
    }

    /**
     * Logs debug information about the motors and sensors in the drivetrain
     */
    public void debugLog() {
        Log.d("Drive Train", "Front Left Distance" + frontLeftEncoder.getDistance());
        Log.d("Drive Train", "Back Left Distance" + backLeftEncoder.getDistance());
        Log.d("Drive Train", "Front Right Distance" + frontRightEncoder.getDistance());
        Log.d("Drive Train", "Back Right Distance" + backRightEncoder.getDistance());
        
        Log.d("Drive Train", "Front Right Temperature "
                        + ((IWPI_TalonSRX) frontRightSwerve.getDriveMotor()).getTemperature());
        Log.d("Drive Train", "Front Left Temperature "
                        + ((IWPI_TalonSRX) frontLeftSwerve.getDriveMotor()).getTemperature());
        Log.d("Drive Train", "Back Right Temperature "
                        + ((IWPI_TalonSRX) backRightSwerve.getDriveMotor()).getTemperature());
        Log.d("Drive Train", "Back Left Temperature "
                        + ((IWPI_TalonSRX) backLeftSwerve.getDriveMotor()).getTemperature());
        
        Log.d("Drive Train", "Front Right Current "
                        + ((IWPI_TalonSRX) frontRightSwerve.getDriveMotor()).getOutputCurrent());
        Log.d("Drive Train", "Front Left Current "
                        + ((IWPI_TalonSRX) frontLeftSwerve.getDriveMotor()).getOutputCurrent());
        Log.d("Drive Train", "Back Right Current "
                        + ((IWPI_TalonSRX) backRightSwerve.getDriveMotor()).getOutputCurrent());
        Log.d("Drive Train", "Back Left Current "
                        + ((IWPI_TalonSRX) backLeftSwerve.getDriveMotor()).getOutputCurrent());
        
        Log.d("Drive Train", "Front Right Output "
                        + ((IWPI_TalonSRX) frontRightSwerve.getDriveMotor()).getMotorOutputPercent());
        Log.d("Drive Train", "Front Left Output "
                        + ((IWPI_TalonSRX) frontLeftSwerve.getDriveMotor()).getMotorOutputPercent());
        Log.d("Drive Train", "Back Right Output "
                        + ((IWPI_TalonSRX) backRightSwerve.getDriveMotor()).getMotorOutputPercent());
        Log.d("Drive Train", "Back Left Output "
                        + ((IWPI_TalonSRX) backLeftSwerve.getDriveMotor()).getMotorOutputPercent());
    }

    /**
     * Resets the sensors on the drivetrain
     */
    public void reset() {
        frontLeftEncoder.reset();
        backLeftEncoder.reset();
        frontRightEncoder.reset();
        backRightEncoder.reset();
    }
    
    /**
     * Swerve drives using the joystick
     * 
     * @param joy The joystick to swerve drive with
     */
    public void swerveDrive(GenericHID joy) {
        double XAxis = RobotMap.INVERT_JOYSTICK_X ? -joy.getRawAxis(0) : joy.getRawAxis(0);
        double YAxis = RobotMap.INVERT_JOYSTICK_Y ? -joy.getRawAxis(1) : joy.getRawAxis(1);
        double ZAxis = RobotMap.INVERT_JOYSTICK_Z ? -joy.getRawAxis(2) : joy.getRawAxis(2);

        swerveDrive(YAxis, XAxis, ZAxis);
    }
    
    /**
     * Swerve drives using input values
     * 
     * @param forward The forward value
     * @param strafe The strafe value
     * @param rotation The rotation value
     */
    public void swerveDrive(double forward, double strafe, double rotation) {
        drive.swerveDrive(forward, strafe, rotation, true);
    }
    
    /**
     * Sets the CANTalon motors to go into brake mode or coast mode
     * 
     * @param on Set to brake when true and coast when false
     */
    public void brakeMode(boolean on) {
        NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;

        Log.i("Brake Mode", mode);

        ((IWPI_TalonSRX) frontLeftSwerve.getDriveMotor()).setNeutralMode(mode);
        ((IWPI_TalonSRX) backLeftSwerve.getDriveMotor()).setNeutralMode(mode);
        ((IWPI_TalonSRX) frontRightSwerve.getDriveMotor()).setNeutralMode(mode);
        ((IWPI_TalonSRX) backRightSwerve.getDriveMotor()).setNeutralMode(mode);
    }
    
    /**
     * Stops the swerve drive control until it is set to a new value
     */
    public void stopSwerve() {
        drive.stopSwerve();
    }
}
