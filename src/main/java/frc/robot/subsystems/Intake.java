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
  SparkMax HotdogmotorID = new SparkMax(Constants.IntakeConstants.HotdogmotorID,MotorType.kBrushless);

  double intakeSpeed = 0.7;

  public Command moveIntake(boolean reverse){
    return run(()->{
      double desiredSpeed = intakeSpeed;
      if(reverse){
          desiredSpeed = -desiredSpeed;
      }
      intakeMax.set(desiredSpeed);

      System.out.println("moveIntake: " + desiredSpeed);
    });
  } 

  public Command intakeIn(){
    return run(()->{
      intakeMax.set(intakeSpeed);
    });
  } 

  public Command intakeOut(){
    return run(()->{
                  HotdogmotorID.set(1);

      intakeMax.set(-intakeSpeed);
    });
  } 

  public Command intakeStop(){
    return run(()->{
                  HotdogmotorID.set(0);

      intakeMax.set(0);
    });
  } 

  public Command moveHotDog(boolean reverse){
    return run(()->{
      double desiredSpeed = 1;
      if(reverse){
        desiredSpeed = -desiredSpeed;
      }
      
      HotdogmotorID.set(desiredSpeed);
    });
  } 

  public Command stopHotDog(){
    return run(()->{
      HotdogmotorID.set(0);
    });
  } 
}
