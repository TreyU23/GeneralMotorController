package frc.robot.Motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class CTREMotor implements Motor {
    private final TalonFX m_motor;

    private final VoltageOut m_voRequest = new VoltageOut(0.0);
    private final DutyCycleOut m_dcRequest = new DutyCycleOut(0.0);

    private final PositionVoltage m_pvRequest = new PositionVoltage(0.0);
    private final VelocityVoltage m_vvRequest = new VelocityVoltage(0.0);

    private final VelocityTorqueCurrentFOC m_vtcRequest = new VelocityTorqueCurrentFOC(0.0);
    private final TorqueCurrentFOC m_tcRequest = new TorqueCurrentFOC(0.0);

    private final MotionMagicVoltage m_mmRequest = new MotionMagicVoltage(0.0);
    private final MotionMagicVelocityVoltage m_mmvvRequest = new MotionMagicVelocityVoltage(0.0);
    private final DynamicMotionMagicVoltage m_dmmRequest = new DynamicMotionMagicVoltage(0.0, 0.0, 0.0);

    public static CTREMotor talonFX(int CanID, MotorConfig config) {
        return new CTREMotor(CanID, config);
    }

    private CTREMotor(int CanID, MotorConfig config) {
        m_motor = new TalonFX(CanID);
        TalonFXConfiguration m_config = buildTalonConfig(config);
        m_motor.getConfigurator().apply(m_config);
    }

    private static TalonFXConfiguration buildTalonConfig(MotorConfig config) {
        TalonFXConfiguration m_config = new TalonFXConfiguration();

        m_config.MotorOutput.Inverted = config.isInverted() ? InvertedValue.Clockwise_Positive
                                                            : InvertedValue.CounterClockwise_Positive;
        m_config.MotorOutput.NeutralMode = config.getIdleMode() == MotorConfig.IdleMode.kBrake ? NeutralModeValue.Brake
                                                                                                : NeutralModeValue.Coast;

        MotorConfig.Gains gains = config.getGains();

        m_config.Slot0.kP = gains.kP();
        m_config.Slot0.kI = gains.kI();
        m_config.Slot0.kD = gains.kD();
        m_config.Slot0.kS = gains.kS();
        m_config.Slot0.kV = gains.kV();
        m_config.Slot0.kG = gains.kG();

        if (!Double.isNaN(config.getSoftLimits().forward())) {
            m_config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.getSoftLimits().forward();
            m_config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        } else {
            m_config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        }

        if (!Double.isNaN(config.getSoftLimits().reverse())) {
            m_config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.getSoftLimits().reverse();
            m_config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        } else {
            m_config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        }

        m_config.MotionMagic.MotionMagicCruiseVelocity = config.getMotionMagic().velocity();
        m_config.MotionMagic.MotionMagicAcceleration = config.getMotionMagic().acceleration();
        m_config.MotionMagic.MotionMagicJerk = config.getMotionMagic().jerk();

        return m_config;
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.setControl(m_dcRequest.withOutput(output));
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setControl(m_voRequest.withOutput(volts));
    }

    @Override
    public void setPosition(double position, boolean FOCEnabled, boolean MotionMagicEnabled) {
        m_motor.setControl(MotionMagicEnabled ? m_mmRequest.withPosition(position).withEnableFOC(FOCEnabled)
                                                : m_pvRequest.withPosition(position).withEnableFOC(FOCEnabled));
    }

    @Override
    public void setVelocity(double velocity, boolean FOCEnabled, boolean MotionMagicEnabled) {
        m_motor.setControl(MotionMagicEnabled ? m_mmvvRequest.withVelocity(velocity).withEnableFOC(FOCEnabled)
                                                : m_vvRequest.withVelocity(velocity).withEnableFOC(FOCEnabled));
    }

    @Override
    public void setDynanmicMotionMagic(double velocity, double position, double acceleration) {
        m_motor.setControl(m_dmmRequest.withVelocity(velocity).withPosition(position).withAcceleration(acceleration));
    }

    @Override
    public void setTorqueCurrent(double stator) {
        m_motor.setControl(m_tcRequest.withOutput(stator));
    }

    @Override
    public void setVelocityTorqueCurrent(double velocity) {
        m_motor.setControl(m_vtcRequest.withVelocity(velocity));
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public double getPosition() {
        return m_motor.getPosition().getValueAsDouble();
    }

    @Override
    public double getVelocity() {
        return m_motor.getVelocity().getValueAsDouble();
    }
}
