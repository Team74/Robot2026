// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public final class Constants
{
  public static final double MaxSystemSpeed = 0.6 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed    
  public static final double MaxSystemAngularRate = RotationsPerSecond.of(0.35).in(RadiansPerSecond); // 1 of a rotation per second max angular velocity

  public static double MAX_SPEED = MaxSystemSpeed;
  public static double MaxAngularRate = MaxSystemAngularRate;
  public static double SlowModeDriveMultiplier = 0.5;
  public static double SlowModeAngleMultiplier = 0.5;
  public static double highModeDriveMultiplier = 1.5;
  public static double highModeAngleMultiplier = 1.5;

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class CurrentLimits {
    public static double DriveStator = 40.0;
    public static double DriveSupply = 40.0;
    public static double SteerStator = 40.0;
    public static double SteerSupply = 40.0;

    public static int ShooterSupplyCurrentLimit = 30;
    public static boolean ShooterSupplyCurrentLimitEnable = true;
    public static int ShooterStatorCurrentLimit = 40;
    public static boolean ShooterStatorCurrentLimitEnable = true;
    public static int Tower = 40;
    public static int Intake = 20;
    public static int IntakeFlipper = 10;
  public static int Hotdog = 0;
    public static int Hood = 10;
  }


  public static final class DrivebaseConstants
  {
    public static final int GyroID = 2;
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class IntakeConstants
  {    
    public static final int HotdogmotorID = 31;
    public static final int MoverMotorID = 44;
    public static final int FeederMotorID = 45;
    public static final double IntakeMoverSpeed = 0.1;
    public static final double HotDogSpeed = 1;
    public static final double intakeSpeed = 1;

    public static final double flipClosedEncoderValue = 0.1;
    public static final double flipOpenEncoderValue = -17.5;
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
    public static double shooterDesiredRPS = 50 ;
    public static int towerDesiredRPS = -90;
    public static double KS = 0.05;
    public static double KV = 0.12;
    public static double KP = 0.1;
    public static double KI = 0.0;
    public static double KD = 0.01;
  }

  public static class ClimberConstants{
    public static final int ClimbMotorID = 30;
    public static final double ClimbSpeed = .7;
  }

  public static class VisionConstants {
    public static final String limelightName = "limelight-bot";

    // public static final double testPoiX = 3.0988; //13.436 for red
    // public static final double testPoiY = 4; //4 for red
    // public static final double testPoiAngle = Math.toRadians(180);

    public static final Pose2d blueShooter = new Pose2d(
      new Translation2d(3.0988, 4.0),
      new Rotation2d(Math.toRadians(180))
    );

    public static final Pose2d redShooter = new Pose2d(
      new Translation2d(13.895, 4.0),
      new Rotation2d(Math.toRadians(0))
    );


    public static final Transform2d shooterRelativeToBot = new Transform2d(
      new Translation2d(0.0, 0.0),
      new Rotation2d(Math.toRadians(180))
    );

    public static final double arcKp = 9.5;
    public static final double arcKi = 0.0;
    public static final double arcKd = 0.23;
  }
  
  public static class FieldTargets {
    public static final Translation2d blueHub = new Translation2d(4.6228, 4);
    public static final Translation2d redHub = new Translation2d(11.912, 4);

  }
}
