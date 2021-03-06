package org.usfirst.frc.team2706.robot.controls.talon;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * Contains all the methods required to run a WPI_TalonSRX. Allows for the use of mock Talons
 */
public interface IWPI_TalonSRX extends IMotorControllerEnhanced, SpeedController {

    public long getHandle();

    /**
     * Returns the Device ID
     *
     * @return Device number.
     */
    public int getDeviceID();

    // ------ Set output routines. ----------//
    /**
     * Sets the appropriate output on the talon, depending on the mode.
     *
     * In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as
     * stopped. In Voltage mode, output value is in volts. In Current mode,
     * output value is in amperes. In Speed mode, output value is in position
     * change / 100ms. In Position mode, output value is in encoder ticks or an
     * analog value, depending on the sensor. In Follower mode, the output value
     * is the integer device ID of the talon to duplicate.
     *
     * @param outputValue
     *            The setpoint value, as described above.
     * @see #SelectProfileSlot to choose between the two sets of gains.
     */
    public void set(ControlMode mode, double outputValue);

    public void set(ControlMode mode, double demand0, double demand1);

    /**
     * Neutral the motor output by setting control mode to disabled.
     */
    public void neutralOutput();

    /**
     * Sets the mode of operation during neutral throttle output.
     *
     * @param neutralMode
     *            The desired mode of operation when the Controller output
     *            throttle is neutral (ie brake/coast)
     **/
    public void setNeutralMode(NeutralMode neutralMode);

    public void enableHeadingHold(boolean enable);

    public void selectDemandType(boolean value);

    // ------ Invert behavior ----------//
    /**
     * Sets the phase of the sensor. Use when controller forward/reverse output
     * doesn't correlate to appropriate forward/reverse reading of sensor.
     *
     * @param PhaseSensor
     *            Indicates whether to invert the phase of the sensor.
     **/
    public void setSensorPhase(boolean PhaseSensor);

    /**
     * Inverts the output of the motor controller. LEDs, sensor phase, and limit
     * switches will also be inverted to match the new forward/reverse
     * directions.
     *
     * @param invert
     *            Invert state to set.
     **/
    public void setInverted(boolean invert);

    public boolean getInverted();

    // ----- general output shaping ------------------//
    /**
     * Configures the open-loop ramp rate of throttle output.
     *
     * @param secondsFromNeutralToFull
     *            Minimum desired time to go from neutral to full throttle. A
     *            value of '0' will disable the ramp.
     * @param timeoutMs
     *            Timeout value in ms. Function will generate error if config is
     *            not successful within timeout.
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configOpenloopRamp(double secondsFromNeutralToFull, int timeoutMs);

    /**
     * Configures the closed-loop ramp rate of throttle output.
     *
     * @param secondsFromNeutralToFull
     *            Minimum desired time to go from neutral to full throttle. A
     *            value of '0' will disable the ramp.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull, int timeoutMs);

    /**
     * Configures the forward peak output percentage.
     *
     * @param percentOut
     *            Desired peak output percentage.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configPeakOutputForward(double percentOut, int timeoutMs);

    /**
     * Configures the reverse peak output percentage.
     *
     * @param percentOut
     *            Desired peak output percentage.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configPeakOutputReverse(double percentOut, int timeoutMs);

    /**
     * Configures the forward nominal output percentage.
     *
     * @param percentOut
     *            Nominal (minimum) percent output.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configNominalOutputForward(double percentOut, int timeoutMs);

    /**
     * Configures the reverse nominal output percentage.
     *
     * @param percentOut
     *            Nominal (minimum) percent output.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configNominalOutputReverse(double percentOut, int timeoutMs);

    /**
     * Configures the output deadband percentage.
     *
     * @param percentDeadband
     *            Desired deadband percentage. Minimum is 0.1%, Maximum is 25%.
     *            Pass 0.04 for 4%.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configNeutralDeadband(double percentDeadband, int timeoutMs);

    // ------ Voltage Compensation ----------//
    /**
     * Configures the Voltage Compensation saturation voltage.
     *
     * @param voltage
     *            TO-DO: Comment me!
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configVoltageCompSaturation(double voltage, int timeoutMs);

    /**
     * Configures the voltage measurement filter.
     *
     * @param filterWindowSamples
     *            Number of samples in the rolling average of voltage
     *            measurement.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configVoltageMeasurementFilter(int filterWindowSamples, int timeoutMs);

    /**
     * Enables voltage compensation. If enabled, voltage compensation works in
     * all control modes.
     *
     * @param enable
     *            Enable state of voltage compensation.
     **/
    public void enableVoltageCompensation(boolean enable);

    // ------ General Status ----------//
    /**
     * Gets the bus voltage seen by the motor controller.
     *
     * @return The bus voltage value (in volts).
     */
    public double getBusVoltage();

    /**
     * Gets the output percentage of the motor controller.
     *
     * @return Output of the motor controller (in percent).
     */
    public double getMotorOutputPercent();

    /**
     * @return applied voltage to motor
     */
    public double getMotorOutputVoltage();

    /**
     * Gets the output current of the motor controller.
     *
     * @return The output current (in amps).
     */
    public double getOutputCurrent();

    /**
     * Gets the temperature of the motor controller.
     *
     * @return Temperature of the motor controller (in 'C)
     */
    public double getTemperature();

    // ------ sensor selection ----------//
    /**
     * Select the remote feedback device for the motor controller.
     *
     * @param feedbackDevice
     *            Remote Feedback Device to select.
     * @param pidIdx
     *            0 for Primary closed-loop. 1 for cascaded closed-loop.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configSelectedFeedbackSensor(RemoteFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs);

    /**
     * Select the feedback device for the motor controller.
     *
     * @param feedbackDevice
     *            Feedback Device to select.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configSelectedFeedbackSensor(FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs);

    public ErrorCode configRemoteFeedbackFilter(int deviceID, RemoteSensorSource remoteSensorSource, int remoteOrdinal,
            int timeoutMs);

    public ErrorCode configSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice, int timeoutMs);

    // ------- sensor status --------- //
    /**
     * Get the selected sensor position.
     *
     * @return Position of selected sensor (in Raw Sensor Units).
     */
    public int getSelectedSensorPosition(int pidIdx);

    /**
     * Get the selected sensor velocity.
     *
     * @return Velocity of selected sensor (in Raw Sensor Units per 100 ms).
     */
    public int getSelectedSensorVelocity(int pidIdx);

    /**
     * Sets the sensor position to the given value.
     *
     * @param sensorPos
     *            Position to set for the selected sensor (in Raw Sensor Units).
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode setSelectedSensorPosition(int sensorPos, int pidIdx, int timeoutMs);

    // ------ status frame period changes ----------//
    /**
     * Sets the period of the given control frame.
     *
     * @param frame
     *            Frame whose period is to be changed.
     * @param periodMs
     *            Period in ms for the given frame.
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode setControlFramePeriod(ControlFrame frame, int periodMs);
    
    /**
     * Sets the period of the given control frame.
     *
     * @param frame
     *            Frame whose period is to be changed.
     * @param periodMs
     *            Period in ms for the given frame.
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode setControlFramePeriod(int frame, int periodMs);
    /**
     * Sets the period of the given status frame.
     *
     * @param frame
     *            Frame whose period is to be changed.
     * @param periodMs
     *            Period in ms for the given frame.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode setStatusFramePeriod(int frameValue, int periodMs, int timeoutMs);

    /**
     * Sets the period of the given status frame.
     *
     * @param frame
     *            Frame whose period is to be changed.
     * @param periodMs
     *            Period in ms for the given frame.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode setStatusFramePeriod(StatusFrame frame, int periodMs, int timeoutMs);

    /**
     * Gets the period of the given status frame.
     *
     * @param frame
     *            Frame to get the period of.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Period of the given status frame.
     */
    public int getStatusFramePeriod(int frame, int timeoutMs);
    
    /**
     * Gets the period of the given status frame.
     *
     * @param frame
     *            Frame to get the period of.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Period of the given status frame.
     */
    public int getStatusFramePeriod(StatusFrame frame, int timeoutMs);

    /**
     * Gets the period of the given status frame.
     *
     * @param frame
     *            Frame to get the period of.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Period of the given status frame.
     */
    public int getStatusFramePeriod(StatusFrameEnhanced frame, int timeoutMs);

    // ----- velocity signal conditionaing ------//

    /**
     * Sets the period over which velocity measurements are taken.
     *
     * @param period
     *            Desired period for the velocity measurement. @see
     *            #VelocityMeasPeriod
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configVelocityMeasurementPeriod(VelocityMeasPeriod period, int timeoutMs);

    /**
     * Sets the number of velocity samples used in the rolling average velocity
     * measurement.
     *
     * @param windowSize
     *            Number of samples in the rolling average of velocity
     *            measurement.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configVelocityMeasurementWindow(int windowSize, int timeoutMs);

    // ------ remote limit switch ----------//
    /**
     * Configures the forward limit switch for a remote source.
     *
     * @param type
     *            Remote limit switch source. @see #LimitSwitchSource
     * @param normalOpenOrClose
     *            Setting for normally open or normally closed.
     * @param deviceID
     *            Device ID of remote source.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configForwardLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
            int deviceID, int timeoutMs);

    /**
     * Configures the reverse limit switch for a remote source.
     *
     * @param type
     *            Remote limit switch source. @see #LimitSwitchSource
     * @param normalOpenOrClose
     *            Setting for normally open or normally closed.
     * @param deviceID
     *            Device ID of remote source.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configReverseLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
            int deviceID, int timeoutMs);

    public ErrorCode configForwardLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
            int timeoutMs);

    /**
     * Sets the enable state for limit switches.
     *
     * @param enable
     *            Enable state for limit switches.
     **/
    public void overrideLimitSwitchesEnable(boolean enable);

    // ------ soft limit ----------//
    /**
     * Configures the forward soft limit threhold.
     *
     * @param forwardSensorLimit
     *            Forward Sensor Position Limit (in Raw Sensor Units).
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configForwardSoftLimitThreshold(int forwardSensorLimit, int timeoutMs);

    /**
     * Configures the reverse soft limit threshold.
     *
     * @param reverseSensorLimit
     *            Reverse Sensor Position Limit (in Raw Sensor Units).
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configReverseSoftLimitThreshold(int reverseSensorLimit, int timeoutMs);   
    
    /**
     * Configures the forward soft limit enable.
     *
     * @param enable
     *            Forward Sensor Position Limit Enable.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configForwardSoftLimitEnable(boolean enable, int timeoutMs);

        /**
     * Configures the reverse soft limit enable.
     *
     * @param enable
     *            Reverse Sensor Position Limit Enable.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configReverseSoftLimitEnable(boolean enable, int timeoutMs);

    /**
     * Sets the enable state for soft limit switches.
     *
     * @param enable
     *            Enable state for soft limit switches.
     **/
    public void overrideSoftLimitsEnable(boolean enable);

    // ------ Current Lim ----------//
    /* not available in base */

    // ------ General Close loop ----------//
    /**
     * Sets the 'P' constant in the given parameter slot.
     *
     * @param slotIdx
     *            Parameter slot for the constant.
     * @param value
     *            Value of the P constant.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode config_kP(int slotIdx, double value, int timeoutMs);

    /**
     * Sets the 'I' constant in the given parameter slot.
     *
     * @param slotIdx
     *            Parameter slot for the constant.
     * @param value
     *            Value of the I constant.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode config_kI(int slotIdx, double value, int timeoutMs);

    /**
     * Sets the 'D' constant in the given parameter slot.
     *
     * @param slotIdx
     *            Parameter slot for the constant.
     * @param value
     *            Value of the D constant.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode config_kD(int slotIdx, double value, int timeoutMs);

    /**
     * Sets the 'F' constant in the given parameter slot.
     *
     * @param slotIdx
     *            Parameter slot for the constant.
     * @param value
     *            Value of the F constant.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode config_kF(int slotIdx, double value, int timeoutMs);

    /**
     * Sets the Integral Zone constant in the given parameter slot.
     *
     * @param slotIdx
     *            Parameter slot for the constant.
     * @param izone
     *            Value of the Integral Zone constant.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode config_IntegralZone(int slotIdx, int izone, int timeoutMs);

    /**
     * Sets the allowable closed-loop error in the given parameter slot.
     *
     * @param slotIdx
     *            Parameter slot for the constant.
     * @param allowableClosedLoopError
     *            Value of the allowable closed-loop error.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configAllowableClosedloopError(int slotIdx, int allowableClosedLoopError, int timeoutMs);

    /**
     * Sets the maximum integral accumulator in the given parameter slot.
     *
     * @param slotIdx
     *            Parameter slot for the constant.
     * @param iaccum
     *            Value of the maximum integral accumulator.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs);

    /**
     * Sets the integral accumulator.
     *
     * @param iaccum
     *            Value to set for the integral accumulator.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode setIntegralAccumulator(double iaccum, int pidIdx, int timeoutMs);

    /**
     * Gets the closed-loop error.
     *
     * @param slotIdx
     *            Parameter slot of the constant.
     * @return Closed-loop error value.
     */
    public int getClosedLoopError(int pidIdx);

    /**
     * Gets the iaccum value.
     *
     * @return Integral accumulator value.
     */
    public double getIntegralAccumulator(int pidIdx);


    /**
     * Gets the derivative of the closed-loop error.
     *
     * @param slotIdx
     *            Parameter slot of the constant.
     * @return The error derivative value.
     */
    public double getErrorDerivative(int pidIdx);

    /**
     * Selects which profile slot to use for closed-loop control.
     *
     * @param slotIdx
     *            Profile slot to select.
     **/
    public void selectProfileSlot(int slotIdx, int pidIdx);

    //public int getClosedLoopTarget(int pidIdx);

    public int getActiveTrajectoryPosition();

    public int getActiveTrajectoryVelocity();

    public double getActiveTrajectoryHeading();

    // ------ Motion Profile Settings used in Motion Magic and Motion Profile ----------//

    /**
     * Sets the Motion Magic Cruise Velocity.
     *
     * @param sensorUnitsPer100ms
     *            Motion Magic Cruise Velocity (in Raw Sensor Units per 100 ms).
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configMotionCruiseVelocity(int sensorUnitsPer100ms, int timeoutMs);

    /**
     * Sets the Motion Magic Acceleration.
     *
     * @param sensorUnitsPer100msPerSec
     *            Motion Magic Acceleration (in Raw Sensor Units per 100 ms per
     *            second).
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configMotionAcceleration(int sensorUnitsPer100msPerSec, int timeoutMs);

    //------ Motion Profile Buffer ----------//
    public ErrorCode clearMotionProfileTrajectories();
    public int getMotionProfileTopLevelBufferCount();
    public ErrorCode pushMotionProfileTrajectory(TrajectoryPoint trajPt);
    public boolean isMotionProfileTopLevelBufferFull();
    public void processMotionProfileBuffer();
    public ErrorCode getMotionProfileStatus(MotionProfileStatus statusToFill);
    public ErrorCode clearMotionProfileHasUnderrun(int timeoutMs);
    public ErrorCode changeMotionControlFramePeriod(int periodMs);
    // ------ error ----------//
    /**
     * Gets the last error generated by this object.
     *
     * @return Last Error Code generated by a function.
     */
    public ErrorCode getLastError();

    // ------ Faults ----------//
    public ErrorCode getFaults(Faults toFill);
    public ErrorCode getStickyFaults(StickyFaults toFill);

    public ErrorCode clearStickyFaults(int timeoutMs);

    // ------ Firmware ----------//
    /**
     * Gets the firmware version of the device.
     *
     * @return Firmware version of device.
     */
    public int getFirmwareVersion();

    /**
     * Returns true if the device has reset.
     *
     * @return Has a Device Reset Occurred?
     */
    public boolean hasResetOccurred();

    //------ Custom Persistent Params ----------//
    /**
     * Sets the value of a custom parameter.
     *
     * @param newValue
     *            Value for custom parameter.
     * @param paramIndex
     *            Index of custom parameter.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configSetCustomParam(int newValue, int paramIndex, int timeoutMs);

    /**
     * Gets the value of a custom parameter.
     *
     * @param paramIndex
     *            Index of custom parameter.
     * @param timoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Value of the custom param.
     */
    public int configGetCustomParam(int paramIndex, int timoutMs);

    // ------ Generic Param API ----------//
    /**
     * Sets a parameter.
     *
     * @param param
     *            Parameter enumeration.
     * @param value
     *            Value of parameter.
     * @param subValue
     *            Subvalue for parameter. Maximum value of 255.
     * @param ordinal
     *            Ordinal of parameter.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configSetParameter(ParamEnum param, double value, int subValue, int ordinal, int timeoutMs);
    /**
     * Sets a parameter.
     *
     * @param param
     *            Parameter enumeration.
     * @param value
     *            Value of parameter.
     * @param subValue
     *            Subvalue for parameter. Maximum value of 255.
     * @param ordinal
     *            Ordinal of parameter.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Error Code generated by function. 0 indicates no error.
     */
    public ErrorCode configSetParameter(int param, double value, int subValue, int ordinal, int timeoutMs);
    /**
     * Gets a parameter.
     *
     * @param param
     *            Parameter enumeration.
     * @param ordinal
     *            Ordinal of parameter.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Value of parameter.
     */
    public double configGetParameter(ParamEnum param, int ordinal, int timeoutMs);
    /**
     * Gets a parameter.
     *
     * @param param
     *            Parameter enumeration.
     * @param ordinal
     *            Ordinal of parameter.
     * @param timeoutMs
     *            Timeout value in ms. @see #ConfigOpenLoopRamp
     * @return Value of parameter.
     */
    public double configGetParameter(int param, int ordinal, int timeoutMs);

    // ------ Misc. ----------//
    public int getBaseID();

    // ----- Follower ------//
    public void follow(IMotorController masterToFollow);

    public void valueUpdated();
    
    /**
     * @retrieve object that can get/set individual RAW sensor values.
     */
    public SensorCollection getSensorCollection();
    
    /**
     * @retrieve control mode of motor controller
     */
    public ControlMode getControlMode();
    
}
