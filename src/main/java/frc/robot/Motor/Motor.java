package frc.robot.Motor;

public interface Motor {
    void setVoltage(double volts);

    void setPosition(double position);

    void setVelocity(double velocity);

    void stop();

    double getPosition();

    double getVelocity();
}
