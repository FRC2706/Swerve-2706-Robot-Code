package org.usfirst.frc.team2706.robot.subsystems;

import org.usfirst.frc.team2706.robot.Log;
import org.usfirst.frc.team2706.robot.RobotMap;
import org.usfirst.frc.team2706.robot.commands.teleop.ArcadeDriveWithJoystick;
import org.usfirst.frc.team2706.robot.controls.swerve.SwerveDrive;
import org.usfirst.frc.team2706.robot.controls.swerve.SwerveModule;
import org.usfirst.frc.team2706.robot.controls.swerve.TalonServo;
import org.usfirst.frc.team2706.robot.controls.talon.TalonEncoder;
import org.usfirst.frc.team2706.robot.controls.talon.TalonSensorGroup;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDriveTrain extends Subsystem {
    private final SwerveModule frontLeftSwerve, backLeftSwerve, frontRightSwerve, backRightSwerve;
    private final TalonEncoder frontLeftEncoder, backLeftEncoder, frontRightEncoder, backRightEncoder;

    private SwerveDrive drive;

    private Command defaultCommand;

    public SwerveDriveTrain() {
        super();

        if (RobotMap.MOTOR_FRONT_LEFT_REAL) {
            WPI_TalonSRX rotate = new WPI_TalonSRX(RobotMap.MOTOR_FRONT_LEFT_ROTATION);
            WPI_TalonSRX drive = new WPI_TalonSRX(RobotMap.MOTOR_FRONT_LEFT_DRIVE);
            frontLeftSwerve = new SwerveModule(drive,
                            new TalonServo(new TalonSensorGroup(rotate, new TalonEncoder(rotate))));
            frontLeftEncoder = new TalonEncoder(drive);
        } else {
            frontLeftSwerve = SwerveDrive.EMPTY;
            frontLeftEncoder = EMPTY;
        }

        if (RobotMap.MOTOR_BACK_LEFT_REAL) {
            WPI_TalonSRX rotate = new WPI_TalonSRX(RobotMap.MOTOR_BACK_LEFT_ROTATION);
            WPI_TalonSRX drive = new WPI_TalonSRX(RobotMap.MOTOR_BACK_LEFT_DRIVE);
            backLeftSwerve = new SwerveModule(drive,
                            new TalonServo(new TalonSensorGroup(rotate, new TalonEncoder(rotate))));
            backLeftEncoder = new TalonEncoder(drive);
        } else {
            backLeftSwerve = SwerveDrive.EMPTY;
            backLeftEncoder = EMPTY;
        }

        if (RobotMap.MOTOR_FRONT_RIGHT_REAL) {
            WPI_TalonSRX rotate = new WPI_TalonSRX(RobotMap.MOTOR_FRONT_RIGHT_ROTATION);
            WPI_TalonSRX drive = new WPI_TalonSRX(RobotMap.MOTOR_FRONT_RIGHT_DRIVE);
            frontRightSwerve = new SwerveModule(drive,
                            new TalonServo(new TalonSensorGroup(rotate, new TalonEncoder(rotate))));
            frontRightEncoder = new TalonEncoder(drive);
        } else {
            frontRightSwerve = SwerveDrive.EMPTY;
            frontRightEncoder = EMPTY;
        }
        
        if (RobotMap.MOTOR_BACK_RIGHT_REAL) {
            WPI_TalonSRX rotate = new WPI_TalonSRX(RobotMap.MOTOR_BACK_RIGHT_ROTATION);
            WPI_TalonSRX drive = new WPI_TalonSRX(RobotMap.MOTOR_BACK_RIGHT_DRIVE);
            backRightSwerve = new SwerveModule(drive,
                            new TalonServo(new TalonSensorGroup(rotate, new TalonEncoder(rotate))));
            backRightEncoder = new TalonEncoder(drive);
            
        } else {
            backRightSwerve = SwerveDrive.EMPTY;
            backRightEncoder = EMPTY;
        }

        drive = new SwerveDrive(frontLeftSwerve, backLeftSwerve, frontRightSwerve, backRightSwerve);


    }

    private static final TalonEncoder EMPTY = new EmptyTalonEncoder();

    private static class EmptyTalonEncoder extends TalonEncoder {

        public EmptyTalonEncoder() {
            super(null);
        }
        
        @Override
        public int get() {
            return 0;
        }
        
        @Override
        public void reset() {}
        
        @Override
        public double getRate() {
            return 0;
        }
        
        public void setDistancePerPulse(double dpp) {}
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
        SmartDashboard.putNumber("Front Left Distance", frontLeftEncoder.getDistance());
        SmartDashboard.putNumber("Back Left Distance", backLeftEncoder.getDistance());
        SmartDashboard.putNumber("Front Right Distance", frontRightEncoder.getDistance());
        SmartDashboard.putNumber("Back Right Distance", backRightEncoder.getDistance());
        SmartDashboard.putNumber("Front Left Speed", frontLeftEncoder.getRate());
        SmartDashboard.putNumber("Back Left Speed", backLeftEncoder.getRate());
        SmartDashboard.putNumber("Front Right Speed", frontRightEncoder.getRate());
        SmartDashboard.putNumber("Back Right Speed", backRightEncoder.getRate());
    }
    
    public void brakeMode(boolean on) {
        if (RobotMap.MOTOR_FRONT_LEFT_REAL) {
            ((WPI_TalonSRX)frontLeftSwerve.getDriveMotor()).setNeutralMode(on ? NeutralMode.Brake : NeutralMode.Coast);
        }
        if (RobotMap.MOTOR_BACK_LEFT_REAL) {
            ((WPI_TalonSRX)backLeftSwerve.getDriveMotor()).setNeutralMode(on ? NeutralMode.Brake : NeutralMode.Coast);
        }
        if (RobotMap.MOTOR_FRONT_RIGHT_REAL) {
            ((WPI_TalonSRX)frontRightSwerve.getDriveMotor()).setNeutralMode(on ? NeutralMode.Brake : NeutralMode.Coast);
        }
        if (RobotMap.MOTOR_BACK_RIGHT_REAL) {
            ((WPI_TalonSRX)backRightSwerve.getDriveMotor()).setNeutralMode(on ? NeutralMode.Brake : NeutralMode.Coast);
        }
    }

    public void reset() {
        frontLeftEncoder.reset();
        backLeftEncoder.reset();
        frontRightEncoder.reset();
        backRightEncoder.reset();
    }
    
    public void swerveDrive(GenericHID joy) {
        double XAxis = RobotMap.INVERT_JOYSTICK_X ? -joy.getRawAxis(0) : joy.getRawAxis(0);
        double YAxis = RobotMap.INVERT_JOYSTICK_Y ? -joy.getRawAxis(1) : joy.getRawAxis(1);
        double ZAxis = RobotMap.INVERT_JOYSTICK_Z ? -joy.getRawAxis(2) : joy.getRawAxis(2);

        swerveDrive(YAxis, XAxis, ZAxis);
    }
    
    public void swerveDrive(double forward, double strafe, double rotation) {
        drive.swerveDrive(forward, strafe, rotation, true);
    }
}
