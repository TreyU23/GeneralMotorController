package frc.robot.Motor;

public interface Motor {
    void setVoltage(double volts);

    void setDutyCycle(double output);

    void setPosition(double position, boolean FOCEnabled, boolean MotionMagicEnabled);

    void setVelocity(double velocity, boolean FOCEnabled, boolean MotionMagicEnabled);

    void setDynanmicMotionMagic(double velocity, double position, double acceleration);

    void setTorqueCurrent(double stator);

    void setVelocityTorqueCurrent(double velocity);

    void stop();

    double getPosition();

    double getVelocity();
}
