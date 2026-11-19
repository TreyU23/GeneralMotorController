package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Motor.Motor;

public class TalonFXSubsystem extends SubsystemBase {
    private final Motor m_motor;

    public TalonFXSubsystem(Motor motor) {
        m_motor = motor;
    }

    public Command setVelocityCmd(double velocity) {
        return runEnd(()-> m_motor.setVelocity(velocity), ()-> m_motor.stop());
    }

    public Command setPositionCmd(double position) {
        return runEnd(()-> m_motor.setPosition(position), ()-> m_motor.stop());
    }

    public Command setVoltageCmd(double volts) {
        return runEnd(()-> m_motor.setVoltage(volts), ()-> m_motor.stop());
    }

    public double getVelocity() {
        return m_motor.getVelocity();
    }

    public double getPosition() {
        return m_motor.getPosition();
    }
}
