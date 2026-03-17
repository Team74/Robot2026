// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public final class Constants
{
  public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  public static final double MaxAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class DrivebaseConstants
  {
    public static final int GyroID = 2;
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class IntakeConstants
  {    
    public static final int HotdogmotorID = 31;
    public static final int MoverMotorID = 34;
    public static final int FeederMotorID = 33;
    public static final double IntakeMoverSpeed = 0.1;
    public static final double HotDogSpeed = 1;
    public static final double intakeSpeed = -1;
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class ShooterConstants{
    public static final int HoodMotorID = 46;
    public static final int TowerMotorID = 3;
    public static final int ShooterMotorID = 6;
    public static final int ShooterMotor2ID = 7;
    public static final double hoodSpeed = .25;
    public static int shooterDesiredRPS = 60;
    public static int towerDesiredRPS = -90;
    public static double KS = 0.05;
    public static double KV = 0.12;
    public static double KP = 0.11;
    public static double KI = 0.5;
    public static double KD = 0.01;
    public static int SupplyCurrentLimit = 40;
    public static boolean SupplyCurrentLimitEnable = true;
    public static int StatorCurrentLimit = 40;
    public static boolean StatorCurrentLimitEnable = true;
  }

  public static class ClimberConstants{
    public static final int ClimbMotorID = 30;
    public static final double ClimbSpeed = .7;
  }

  public static class VisionConstants {
    public static final String limelightName = "limelight-bot";

    public static final double testPoiX = 2.33;
    public static final double testPoiY = 4;
    public static final double testPoiAngle = Math.toRadians(0);

  }
}
