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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  TalonFX shooterMotor = new TalonFX(Constants.ShooterConstants.ShooterMotorID);
  SparkMax hoodMotor = new SparkMax(Constants.ShooterConstants.HoodMotorID, MotorType.kBrushless); 
  SparkMax towerMotor = new SparkMax(Constants.ShooterConstants.TowerMotorID, MotorType.kBrushless);

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

  public ShooterSubsystem() {
    shooterMotor.getConfigurator().apply(toConfigure);
    shooterMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public Command shoot(){
    return run(()->{
      var request = new VelocityVoltage(0).withSlot(0);
      shooterMotor.setControl(request.withVelocity(Constants.ShooterConstants.desiredRPS).withFeedForward(0.5));
      towerMotor.set(1);
    });
  } 

  public Command stopShooter(){
    return run(()->{
      var request = new VelocityVoltage(0).withSlot(0);
      shooterMotor.setControl(request.withVelocity(0).withFeedForward(0.5));
      towerMotor.set(0);
    });
  } 

  public Command MoveHoodOut(){
    return run(()->{
      hoodMotor.set(0.25);
    });
  }

  public Command MoveHoodIn(){
    return run(()->{
      hoodMotor.set(-0.25);
    });
  }

  public Command StopHood(){
    return run(()->{
      hoodMotor.set(0);
    });
  }
}
