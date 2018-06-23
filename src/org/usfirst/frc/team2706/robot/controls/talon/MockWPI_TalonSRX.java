package org.usfirst.frc.team2706.robot.controls.talon;

import java.lang.reflect.Field;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
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

import sun.misc.Unsafe;

public class MockWPI_TalonSRX implements IWPI_TalonSRX {

    @Override
    public ErrorCode setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(LimitSwitchSource type,
                    LimitSwitchNormal normalOpenOrClose, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configPeakCurrentLimit(int amps, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configPeakCurrentDuration(int milliseconds, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configContinuousCurrentLimit(int amps, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public void enableCurrentLimit(boolean enable) {}

    @Override
    public long getHandle() {
        return 0;
    }

    @Override
    public int getDeviceID() {
        return 0;
    }

    @Override
    public void set(ControlMode mode, double outputValue) {}

    @Override
    public void set(ControlMode mode, double demand0, double demand1) {}

    @Override
    public void neutralOutput() {}

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {}

    @Override
    public void enableHeadingHold(boolean enable) {}

    @Override
    public void selectDemandType(boolean value) {}

    @Override
    public void setSensorPhase(boolean PhaseSensor) {}

    @Override
    public void setInverted(boolean invert) {}

    @Override
    public boolean getInverted() {
        return false;
    }

    @Override
    public ErrorCode configOpenloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configPeakOutputForward(double percentOut, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configPeakOutputReverse(double percentOut, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configNominalOutputForward(double percentOut, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configNominalOutputReverse(double percentOut, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configNeutralDeadband(double percentDeadband, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configVoltageCompSaturation(double voltage, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configVoltageMeasurementFilter(int filterWindowSamples, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public void enableVoltageCompensation(boolean enable) {}

    @Override
    public double getBusVoltage() {
        return 0;
    }

    @Override
    public double getMotorOutputPercent() {
        return 0;
    }

    @Override
    public double getMotorOutputVoltage() {
        return 0;
    }

    @Override
    public double getOutputCurrent() {
        return 0;
    }

    @Override
    public double getTemperature() {
        return 0;
    }

    @Override
    public ErrorCode configSelectedFeedbackSensor(RemoteFeedbackDevice feedbackDevice, int pidIdx,
                    int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configSelectedFeedbackSensor(FeedbackDevice feedbackDevice, int pidIdx,
                    int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(int deviceID, RemoteSensorSource remoteSensorSource,
                    int remoteOrdinal, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice,
                    int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public int getSelectedSensorPosition(int pidIdx) {
        return 0;
    }

    @Override
    public int getSelectedSensorVelocity(int pidIdx) {
        return 0;
    }

    @Override
    public ErrorCode setSelectedSensorPosition(int sensorPos, int pidIdx, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode setControlFramePeriod(ControlFrame frame, int periodMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode setControlFramePeriod(int frame, int periodMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode setStatusFramePeriod(int frameValue, int periodMs, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode setStatusFramePeriod(StatusFrame frame, int periodMs, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public int getStatusFramePeriod(int frame, int timeoutMs) {
        return 0;
    }

    @Override
    public int getStatusFramePeriod(StatusFrame frame, int timeoutMs) {
        return 0;
    }

    @Override
    public int getStatusFramePeriod(StatusFrameEnhanced frame, int timeoutMs) {
        return 0;
    }

    @Override
    public ErrorCode configVelocityMeasurementPeriod(VelocityMeasPeriod period, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configVelocityMeasurementWindow(int windowSize, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configForwardLimitSwitchSource(RemoteLimitSwitchSource type,
                    LimitSwitchNormal normalOpenOrClose, int deviceID, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(RemoteLimitSwitchSource type,
                    LimitSwitchNormal normalOpenOrClose, int deviceID, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configForwardLimitSwitchSource(LimitSwitchSource type,
                    LimitSwitchNormal normalOpenOrClose, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public void overrideLimitSwitchesEnable(boolean enable) {}

    @Override
    public ErrorCode configForwardSoftLimitThreshold(int forwardSensorLimit, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configReverseSoftLimitThreshold(int reverseSensorLimit, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configForwardSoftLimitEnable(boolean enable, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configReverseSoftLimitEnable(boolean enable, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public void overrideSoftLimitsEnable(boolean enable) {}

    @Override
    public ErrorCode config_kP(int slotIdx, double value, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode config_kI(int slotIdx, double value, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode config_kD(int slotIdx, double value, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode config_kF(int slotIdx, double value, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode config_IntegralZone(int slotIdx, int izone, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configAllowableClosedloopError(int slotIdx, int allowableClosedLoopError,
                    int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode setIntegralAccumulator(double iaccum, int pidIdx, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public int getClosedLoopError(int pidIdx) {
        return 0;
    }

    @Override
    public double getIntegralAccumulator(int pidIdx) {
        return 0;
    }

    @Override
    public double getErrorDerivative(int pidIdx) {
        return 0;
    }

    @Override
    public void selectProfileSlot(int slotIdx, int pidIdx) {}

    @Override
    public int getActiveTrajectoryPosition() {
        return 0;
    }

    @Override
    public int getActiveTrajectoryVelocity() {
        return 0;
    }

    @Override
    public double getActiveTrajectoryHeading() {
        return 0;
    }

    @Override
    public ErrorCode configMotionCruiseVelocity(int sensorUnitsPer100ms, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configMotionAcceleration(int sensorUnitsPer100msPerSec, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode clearMotionProfileTrajectories() {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public int getMotionProfileTopLevelBufferCount() {
        return 0;
    }

    @Override
    public ErrorCode pushMotionProfileTrajectory(TrajectoryPoint trajPt) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public boolean isMotionProfileTopLevelBufferFull() {
        return false;
    }

    @Override
    public void processMotionProfileBuffer() {}

    @Override
    public ErrorCode getMotionProfileStatus(MotionProfileStatus statusToFill) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode clearMotionProfileHasUnderrun(int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode changeMotionControlFramePeriod(int periodMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode getLastError() {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode getFaults(Faults toFill) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode getStickyFaults(StickyFaults toFill) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode clearStickyFaults(int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public int getFirmwareVersion() {
        return 0;
    }

    @Override
    public boolean hasResetOccurred() {
        return false;
    }

    @Override
    public ErrorCode configSetCustomParam(int newValue, int paramIndex, int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public int configGetCustomParam(int paramIndex, int timoutMs) {
        return 0;
    }

    @Override
    public ErrorCode configSetParameter(ParamEnum param, double value, int subValue, int ordinal,
                    int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public ErrorCode configSetParameter(int param, double value, int subValue, int ordinal,
                    int timeoutMs) {
        return ErrorCode.FeatureNotSupported;
    }

    @Override
    public double configGetParameter(ParamEnum param, int ordinal, int timeoutMs) {
        return 0;
    }

    @Override
    public double configGetParameter(int param, int ordinal, int timeoutMs) {
        return 0;
    }

    @Override
    public int getBaseID() {
        return 0;
    }

    @Override
    public void follow(IMotorController masterToFollow) {}

    @Override
    public void valueUpdated() {}

    private final SensorCollection collection;

    {
        MockSensorCollection temp = null;

        try {
            Field unsafeField = Unsafe.class.getDeclaredField("theUnsafe");
            unsafeField.setAccessible(true);
            Unsafe unsafe = (Unsafe) unsafeField.get(null);
            temp = (MockSensorCollection) unsafe.allocateInstance(MockSensorCollection.class);
            unsafeField.setAccessible(false);
        } catch (InstantiationException | NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e) {
            e.printStackTrace();
        }

        collection = temp;
    }

    @Override
    public SensorCollection getSensorCollection() {
        return collection;
    }

    @Override
    public ControlMode getControlMode() {
        return ControlMode.Disabled;
    }

    private class MockSensorCollection extends SensorCollection {

        public MockSensorCollection() {
            super(null);
        }

        public int getAnalogIn() {
            return 0;
        }

        public ErrorCode setAnalogPosition(int newPosition, int timeoutMs) {
            return ErrorCode.FeatureNotSupported;
        }

        public int getAnalogInRaw() {
            return 0;
        }

        public int getAnalogInVel() {
            return 0;
        }

        public int getQuadraturePosition() {
            return 0;
        }

        public ErrorCode setQuadraturePosition(int newPosition, int timeoutMs) {
            return ErrorCode.FeatureNotSupported;
        }

        public int getQuadratureVelocity() {
            return 0;
        }

        public int getPulseWidthPosition() {
            return 0;
        }

        public ErrorCode setPulseWidthPosition(int newPosition, int timeoutMs) {
            return ErrorCode.FeatureNotSupported;
        }

        public int getPulseWidthVelocity() {
            return 0;
        }

        public int getPulseWidthRiseToFallUs() {
            return 0;
        }

        public int getPulseWidthRiseToRiseUs() {
            return 0;
        }

        public boolean getPinStateQuadA() {
            return false;
        }


        public boolean getPinStateQuadB() {
            return false;
        }

        public boolean getPinStateQuadIdx() {
            return false;
        }

        public boolean isFwdLimitSwitchClosed() {
            return false;
        }

        public boolean isRevLimitSwitchClosed() {
            return false;
        }
    }

    @Override
    public void pidWrite(double output) {}

    @Override
    public void set(double speed) {}

    @Override
    public double get() {
        return 0;
    }

    @Override
    public void disable() {}

    @Override
    public void stopMotor() {}
}
