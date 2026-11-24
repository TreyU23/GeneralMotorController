package frc.robot.Motor;

public class MotorConfig {
    public enum IdleMode {
        kBrake,
        kCoast
    }

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kG) {
        public static Gains none() {
            return new Gains(0, 0, 0, 0, 0, 0);
        }

        public static Gains pid(double kP, double kI, double kD) {
            return new Gains(kP, kI, kD, 0, 0, 0);
        }

        public static Gains all(double kP, double kI, double kD, double kS, double kV, double kG) {
            return new Gains(kP, kI, kD, kS, kV, kG);
        }
    }

    public record SoftLimits(double forward, double reverse) {
        public static SoftLimits none() {
            return new SoftLimits(Double.NaN, Double.NaN);
        }

        public static SoftLimits both(double forward, double reverse) {
            return new SoftLimits(forward, reverse);
        }
    }

    public record MotionMagic(double velocity, double acceleration, double jerk) {
        public static MotionMagic none() {
            return new MotionMagic(0, 0, 0);
        }

        public static MotionMagic all(double velocity, double acceleration, double jerk) {
            return new MotionMagic(velocity, acceleration, jerk);
        }
    }

    private final boolean m_isInverted;
    private final IdleMode m_idleMode;
    private final Gains m_gains;
    private final SoftLimits m_softLimits;
    private final MotionMagic m_motionMagic;

    public MotorConfig(boolean isInverted, IdleMode idleMode, Gains gains, SoftLimits softLimits, MotionMagic motionMagic) {
        m_isInverted = isInverted;
        m_idleMode = idleMode;
        m_gains = gains;
        m_softLimits = softLimits;
        m_motionMagic = motionMagic;
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

    public MotionMagic getMotionMagic() {
        return m_motionMagic;
    }
}
