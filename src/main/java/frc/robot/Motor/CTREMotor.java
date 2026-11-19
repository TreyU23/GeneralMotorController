package frc.robot.Motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class CTREMotor implements Motor {
    private final TalonFX m_motor;

    private final VoltageOut m_voRequest = new VoltageOut(0.0);
    private final PositionVoltage m_pvRequest = new PositionVoltage(0.0);
    private final VelocityVoltage m_vvRequest = new VelocityVoltage(0.0);

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

        m_config.MotorOutput.Inverted = config.isInverted() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        m_config.MotorOutput.NeutralMode = config.getIdleMode() == MotorConfig.IdleMode.kBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        MotorConfig.Gains gains = config.getGains();

        m_config.Slot0.kP = gains.kP();
        m_config.Slot0.kI = gains.kI();
        m_config.Slot0.kD = gains.kD();
        m_config.Slot0.kS = gains.kS();
        m_config.Slot0.kV = gains.kV();

        m_config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.getSoftLimits().forward();
        m_config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.getSoftLimits().reverse();

        m_config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        return m_config;
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setControl(m_voRequest.withOutput(volts));
    }

    @Override
    public void setPosition(double position) {
        m_motor.setControl(m_pvRequest.withPosition(position));
    }

    @Override
    public void setVelocity(double velocity) {
        m_motor.setControl(m_vvRequest.withVelocity(velocity));
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
