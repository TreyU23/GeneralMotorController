package frc.robot.Motor;

public class MotorConfig {
    public enum IdleMode {
        kBrake,
        kCoast
    }

    public record Gains(double kP, double kI, double kD, double kS, double kV) {
        public static Gains none() {
            return new Gains(0, 0, 0, 0, 0);
        }

        public static Gains pid(double kP, double kI, double kD) {
            return new Gains(kP, kI, kD, 0, 0);
        }
    }

    public record SoftLimits(double forward, double reverse) {
        public static SoftLimits both(double forward, double reverse) {
            return new SoftLimits(forward, reverse);
        }
    }

    private final boolean m_isInverted;
    private final IdleMode m_idleMode;
    private final Gains m_gains;
    private final SoftLimits m_softLimits;

    public MotorConfig(boolean isInverted, IdleMode idleMode, Gains gains, SoftLimits softLimits) {
        m_isInverted = isInverted;
        m_idleMode = idleMode;
        m_gains = gains;
        m_softLimits = softLimits;
    }

    public boolean isInverted() {
        return m_isInverted;
    }

    public IdleMode getIdleMode() {
        return m_idleMode;
    }

    public Gains getGains() {
        return m_gains;
    }

    public SoftLimits getSoftLimits() {
        return m_softLimits;
    }
}
