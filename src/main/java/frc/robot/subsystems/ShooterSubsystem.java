// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    TalonFX shooterMotor = new TalonFX(Constants.ShooterConstants.shooterMotorId);

    CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs()
      .withSupplyCurrentLimit(40)
      .withSupplyCurrentLimitEnable(true)
      .withStatorCurrentLimit(40)
      .withStatorCurrentLimitEnable(true);

    Slot0Configs slot0Configs = new Slot0Configs()
      .withKS(0.05)
      .withKV(0.12)
      .withKP(0.11)
      .withKI(0.5)
      .withKD(0.01);

    TalonFXConfiguration toConfigure = new TalonFXConfiguration()
      .withCurrentLimits(m_currentLimits)
      .withSlot0(slot0Configs);

    VelocityVoltage m_velocityVoltage = new VelocityVoltage(0)
      .withSlot(0);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterMotor.getConfigurator().apply(toConfigure);
    shooterMotor.setNeutralMode(NeutralModeValue.Brake);
  }
    
  // public Command setVelocity(double speed) {
  //   //return shooterMotor.setControl(m_velocityVoltage.withVelocity(speed));
  // }



  public Command ShooterMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An Shooter method querying a boolean state of the subsystem (for Shooter, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean ShooterCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
