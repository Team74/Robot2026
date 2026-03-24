package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.units.AngularVelocityUnit;

import static edu.wpi.first.units.Units.RPM;

import java.lang.ModuleLayer.Controller;
import java.util.List;

import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {
  TalonFX shooterMotor = new TalonFX(Constants.ShooterConstants.ShooterMotorID);
  TalonFX shooterMotor2 = new TalonFX(Constants.ShooterConstants.ShooterMotor2ID);
  SparkMax towerMotor = new SparkMax(Constants.ShooterConstants.TowerMotorID, MotorType.kBrushless); 
  SparkMax hotdogMotor = new SparkMax(Constants.IntakeConstants.HotdogmotorID,MotorType.kBrushless);

  Follower thign = new Follower(6, MotorAlignmentValue.Opposed);

  double desiredShootSpeed = Constants.ShooterConstants.shooterDesiredRPS; 
  int desiredTowerSpeed = Constants.ShooterConstants.towerDesiredRPS; 
  double hotdogSpeed = Constants.IntakeConstants.HotDogSpeed;
  static int i = 0;
  public double currentRPS_Shooter = shooterMotor.getVelocity().getValueAsDouble();

  CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(Constants.CurrentLimits.SupplyCurrentLimit)
    .withSupplyCurrentLimitEnable(Constants.CurrentLimits.SupplyCurrentLimitEnable)
    .withStatorCurrentLimit(Constants.CurrentLimits.StatorCurrentLimit)
    .withStatorCurrentLimitEnable(Constants.CurrentLimits.StatorCurrentLimitEnable);

  Slot0Configs slot0Configs = new Slot0Configs()
    .withKS(Constants.ShooterConstants.KS)
    .withKV(Constants.ShooterConstants.KV)
    .withKP(Constants.ShooterConstants.KP)
    .withKI(Constants.ShooterConstants.KI)
    .withKD(Constants.ShooterConstants.KD);

  TalonFXConfiguration toConfigure = new TalonFXConfiguration()
    .withCurrentLimits(m_currentLimits)
    .withSlot0(slot0Configs);

  VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

  public Shooter() {
    shooterMotor.getConfigurator().apply(toConfigure);
    shooterMotor.setNeutralMode(NeutralModeValue.Coast);
    shooterMotor2.getConfigurator().apply(toConfigure);
    shooterMotor2.setNeutralMode(NeutralModeValue.Coast);
    shooterMotor2.setControl(thign);

    SparkMaxConfig towerMotorConfig = new SparkMaxConfig();

    towerMotorConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(40);
        
    towerMotor.configure(towerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

  }

  public double pulse(int FullCycleLength){
    if (i == FullCycleLength){
      i = 0;
      return 0;
    } else if (i > (FullCycleLength/2)) {
      i++;
      return 0;
    } else {
      i++;
      return 1;
    }
  }

  public Command shoot(){
    return run(()->{
      currentRPS_Shooter = shooterMotor.getVelocity().getValueAsDouble();   

      //System.out.println("currentRPS_Shooter: " + currentRPS_Shooter);

      SmartDashboard.putNumber("ShooterRPS", currentRPS_Shooter);
      SmartDashboard.putNumber("Shooter desiredSpeed", desiredShootSpeed);

      shooterMotor.setControl(m_velocityVoltage.withVelocity(desiredShootSpeed).withFeedForward(0.5));

      if (currentRPS_Shooter >= desiredShootSpeed * 0.9) {
        towerMotor.set(desiredTowerSpeed * -1);
        hotdogMotor.set(hotdogSpeed);
      } else {
        towerMotor.set(0);
        hotdogMotor.set(0);
      }
    });
  } 

  public Command reverseShoot(){
    return run(()->{
      currentRPS_Shooter = shooterMotor.getVelocity().getValueAsDouble();
      var reverseShooterSpeed = desiredShootSpeed * -1;       

      shooterMotor.setControl(m_velocityVoltage.withVelocity(reverseShooterSpeed).withFeedForward(0.5));

      if (currentRPS_Shooter <= desiredShootSpeed * 0.75) {
        towerMotor.set(desiredTowerSpeed);
        hotdogMotor.set(hotdogSpeed * -1);
      }
    });
  } 

  public Command stopShooter(){
  
    return run(()->{
      shooterMotor.setControl(m_velocityVoltage.withVelocity(0).withFeedForward(0.5));
      towerMotor.set(0);
      hotdogMotor.set(0);
    });
  } 

  public Command SpinUpToRPS(){
    return run(()->{
      shooterMotor.setControl(m_velocityVoltage.withVelocity(desiredShootSpeed).withFeedForward(0.5));
    });
  }
  
  public Command testTower(boolean reverse){
    return run(()->{
      var desiredSpeed = Constants.ShooterConstants.shooterDesiredRPS;
      if(reverse){
        desiredSpeed = -desiredSpeed;
      }
      towerMotor.set(desiredSpeed);    
    });
  } 
}
