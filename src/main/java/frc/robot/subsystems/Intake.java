package frc.robot.subsystems;

import java.lang.invoke.VarHandle;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
  SparkMax intakeMax = new SparkMax(Constants.IntakeConstants.FeederMotorID, MotorType.kBrushless);
  SparkMax hotdogMotor = new SparkMax(Constants.IntakeConstants.HotdogmotorID,MotorType.kBrushless);

  double intakeDesiredSpeed = Constants.IntakeConstants.intakeSpeed;
  double hotdogSpeed = Constants.IntakeConstants.HotDogSpeed;


  public Command intakeIn(){
    return run(()->{
      intakeMax.set(intakeDesiredSpeed);
      hotdogMotor.set(hotdogSpeed * 0.5);
    });
  } 

  public Command intakeOut(){
    return run(()->{
      intakeMax.set(intakeDesiredSpeed * -1);
      hotdogMotor.set(hotdogSpeed * -0.5);
    });
  } 

  public Command intakeStop(){
    return run(()->{
      hotdogMotor.set(0);
      intakeMax.set(0);
    });
  } 

  public Command hotdogTest(){
    return run(()->{
      hotdogMotor.set(hotdogSpeed);
    });
  }
}
