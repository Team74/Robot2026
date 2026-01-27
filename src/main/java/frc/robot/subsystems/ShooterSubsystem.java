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
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    TalonFX shooterMotor = new TalonFX(Constants.ShooterConstants.shooterMotorId);

    CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs()
      .withSupplyCurrentLimit(Constants.ShooterConstants.SupplyCurrentLimit)
      .withSupplyCurrentLimitEnable(Constants.ShooterConstants.SupplyCurrentLimitEnable)
      .withStatorCurrentLimit(Constants.ShooterConstants.StatorCurrentLimit)
      .withStatorCurrentLimitEnable(Constants.ShooterConstants.StatorCurrentLimitEnable);

    Slot0Configs slot0Configs = new Slot0Configs()
      .withKS(Constants.ShooterConstants.KS)
      .withKV(Constants.ShooterConstants.KV)
      .withKP(Constants.ShooterConstants.KP)
      .withKI(Constants.ShooterConstants.KI)
      .withKD(Constants.ShooterConstants.KD);

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
    
  public Command setVelocity(double speed) {
    var request = new VelocityVoltage(0).withSlot(0);

    return startEnd(
      () -> shooterMotor.setControl(request.withVelocity(speed).withFeedForward(0.5)), //Button pressed
      () -> shooterMotor.setControl(request.withVelocity(0).withFeedForward(0.5)) //Button Released
    );
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
