// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Motor.CTREMotor;
import frc.robot.Motor.Motor;
import frc.robot.Motor.MotorConfig;
import frc.robot.Motor.MotorConfig.*;
import frc.robot.Subsystems.TalonFXSubsystem;

public class RobotContainer {
  private final CommandXboxController m_driver = new CommandXboxController(0);

  private enum Vendor {
    CTRE,
    REV
  }

  private final TalonFXSubsystem m_subsytem = new TalonFXSubsystem(createMotor(1, m_config, Vendor.CTRE));

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driver.rightTrigger().whileTrue(m_subsytem.setVoltageCmd(10));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private static final MotorConfig m_config = new MotorConfig(false, IdleMode.kCoast, Gains.pid(1.0, 0, 0), SoftLimits.both(10, 0));

  private static Motor createMotor(int CanID, MotorConfig config, Vendor vendor) {
    if (vendor == Vendor.CTRE) return CTREMotor.talonFX(CanID, config);
    else return null;
  }
}
