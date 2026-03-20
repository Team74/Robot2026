package frc.robot.subsystems;

import java.lang.invoke.VarHandle;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeFlipper extends SubsystemBase{
  public enum eDesiredEndState {
    IN,
    OUT
  }
  public enum eCurrentState {
    IN_STOPPED,
    OUT_STOPPED,
    MOVING_IN,
    MOVING_OUT
  }

  double desiredPositionTarget = 0;

  public eCurrentState currentState = eCurrentState.IN_STOPPED; 
  public eDesiredEndState currentDesiredState = eDesiredEndState.IN; 

  SparkMax intakeMoverMax = new SparkMax(Constants.IntakeConstants.MoverMotorID, MotorType.kBrushless);

  DigitalInput m_toplimitswitch = new DigitalInput(1);
  DigitalInput m_bottomlimitswitch = new DigitalInput(0);

  double intakeMoverSpeedConstant = Constants.IntakeConstants.IntakeMoverSpeed;
  double desiredintakeMoverSpeed = 0;

  private static double kS = 1.1;
  private static double kG = 1.2;
  private static double kV = 1.3;

  ProfiledPIDController pidFlipPidController = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(1, 5));
  //ArmFeedforward flipFeedforward = new ArmFeedforward(kS, kG, kV);

  double currentPosition = 0;
  double flipperMotorSpeed = 0;

  public IntakeFlipper() {
    intakeMoverMax.getEncoder().setPosition(0);
    
    pidFlipPidController.disableContinuousInput();

    SparkMaxConfig intakeFlipperConfig = new SparkMaxConfig();

    intakeFlipperConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Constants.IntakeConstants.flipSmartCurrentLimit);
        
    intakeMoverMax.configure(intakeFlipperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command IntakeIn(){
     return runOnce( () -> {
        currentDesiredState = eDesiredEndState.IN;
      });
  }

  public Command IntakeOut(){
     return runOnce( () -> {
        currentDesiredState = eDesiredEndState.OUT;
      });
  }

  public Command SwapDesiredState(){
    return runOnce( () -> {
      if(currentDesiredState == eDesiredEndState.IN) {
        currentDesiredState = eDesiredEndState.OUT;
      }
      else {
        currentDesiredState = eDesiredEndState.IN;
      }
    });
  }

  public Command MoveToDesiredState(){
    return run( () -> {
      var isTopPressed = m_toplimitswitch.get();
      var isBottomPressed = m_bottomlimitswitch.get();

      if(isTopPressed) {
          intakeMoverMax.getEncoder().setPosition(Constants.IntakeConstants.flipClosedEncoderValue);
          currentState = eCurrentState.IN_STOPPED;
      }
      if(isBottomPressed) {
          intakeMoverMax.getEncoder().setPosition(Constants.IntakeConstants.flipOpenEncoderValue);
          currentState = eCurrentState.OUT_STOPPED;
      }
      if(currentDesiredState == eDesiredEndState.IN) {
        desiredPositionTarget = Constants.IntakeConstants.flipClosedEncoderValue;
      } 
      if(currentDesiredState == eDesiredEndState.OUT) {
        desiredPositionTarget = Constants.IntakeConstants.flipOpenEncoderValue;
      }

      flipperMotorSpeed = 
        pidFlipPidController.calculate(currentPosition, desiredPositionTarget);
        
      if(flipperMotorSpeed > 0) {
          currentState = eCurrentState.MOVING_IN;
      } 
      else if(flipperMotorSpeed < 0) {
          currentState = eCurrentState.MOVING_OUT;
      }

      intakeMoverMax.set(flipperMotorSpeed);
    });
  }

  public Command Stop(){
    return run( () -> {
      intakeMoverMax.set(0);
    });
  }

  @Override
  public void periodic() {
    currentPosition = intakeMoverMax.getEncoder().getPosition();

    SmartDashboard.putNumber("Flipper Angle", currentPosition);
    SmartDashboard.putNumber("Flipper Motor Speed", flipperMotorSpeed);
    SmartDashboard.putString("Flipper DesiredState", currentDesiredState.toString());
    SmartDashboard.putString("Flipper Current State", currentState.toString());
  }
}
