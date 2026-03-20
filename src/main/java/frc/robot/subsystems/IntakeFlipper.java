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
import edu.wpi.first.math.controller.PIDController;
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
  SparkClosedLoopController m_controller = intakeMoverMax.getClosedLoopController();
  private SparkMaxConfig motorConfig;

  DigitalInput m_toplimitswitch = new DigitalInput(1);
  DigitalInput m_bottomlimitswitch = new DigitalInput(0);

  double intakeMoverSpeedConstant = Constants.IntakeConstants.IntakeMoverSpeed;
  double desiredintakeMoverSpeed = 0;

  public IntakeFlipper() {
    intakeMoverMax.getEncoder().setPosition(0);

    motorConfig = new SparkMaxConfig();
    
    motorConfig.encoder
      .positionConversionFactor(0.5)
      .velocityConversionFactor(1);

    motorConfig.closedLoop
      
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .p(0.2)
      .i(0)
      .d(0)
      .outputRange(-1, 1)
      // Set PID values for velocity control in slot 1
      .p(0.0001, ClosedLoopSlot.kSlot1)
      .i(0, ClosedLoopSlot.kSlot1)
      .d(0, ClosedLoopSlot.kSlot1)
      .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
      .feedForward
      // kV is now in Volts, so we multiply by the nominal voltage (12V)
      .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

      motorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .cruiseVelocity(2000)
        .maxAcceleration(2000)
        .allowedProfileError(1)
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(1000, ClosedLoopSlot.kSlot1)
        .cruiseVelocity(6000, ClosedLoopSlot.kSlot1)
        .allowedProfileError(1, ClosedLoopSlot.kSlot1);

      intakeMoverMax.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public Command SwapDesiredState(){
    return runOnce( () -> {
      if(currentDesiredState == eDesiredEndState.IN) {
        currentDesiredState = eDesiredEndState.OUT;
      }
      else {
        currentDesiredState = eDesiredEndState.IN;
      }
      System.out.println("swap");
    });
  }

  public Command MoveToDesiredState_Old(){
    return run( () -> {
      var isTopPressed = m_toplimitswitch.get();
      var isBottomPressed = m_bottomlimitswitch.get();

      //If both limits are pressed.... Prob should check the bot....
      if(isTopPressed && isBottomPressed) {
        //Stop all
        desiredintakeMoverSpeed = 0;
      }
      //If the top limit is hit but not the bottom
      else if(isTopPressed && !isBottomPressed) {
        if(currentDesiredState == eDesiredEndState.IN) {
          //Top is hit and you want to bring it back IN
          //Send 0
          desiredintakeMoverSpeed = 0;
          currentState = eCurrentState.IN_STOPPED;
        }
        if(currentDesiredState == eDesiredEndState.OUT) {
          //Top is hit and you want to bring it back OUT
          //Send neg value
          desiredintakeMoverSpeed = -intakeMoverSpeedConstant;
          currentState = eCurrentState.MOVING_OUT;
        }
      }
      //If the top limit is NOT hit but the bottom limit IS
      else if(!isTopPressed && isBottomPressed) {
        if(currentDesiredState == eDesiredEndState.IN) {
          //Bottom is hit so make sure to give motor a positive value.
          //Send pos val
          desiredintakeMoverSpeed = intakeMoverSpeedConstant;
          currentState = eCurrentState.MOVING_IN;
        }
        if(currentDesiredState == eDesiredEndState.OUT) {
          //Bottom is hit so make sure to give motor a positive value.
          //Send 0
          desiredintakeMoverSpeed = 0;
          currentState = eCurrentState.OUT_STOPPED;
        }
      }
      //If both limits are not hit.
      else {
        //Do nothing keep moving in the previous direction until one of the limits are tripped  
      }

      intakeMoverMax.set(desiredintakeMoverSpeed);
      System.out.println ("isTopPressed: " + isTopPressed + " isBottomPressed: " + isBottomPressed + " desiredintakeMoverSpeed: " + desiredintakeMoverSpeed + " currentState: " + currentState + " currentDesiredState: " + currentDesiredState);
    });
  }

  public Command MoveToDesiredState(){
    return run( () -> {
    
        SmartDashboard.putNumber("Flipper Angle", intakeMoverMax.getEncoder().getPosition());


      var isTopPressed = m_toplimitswitch.get();
      var isBottomPressed = m_bottomlimitswitch.get();

      if(isTopPressed) {
          intakeMoverMax.getEncoder().setPosition(0);
          currentState = eCurrentState.IN_STOPPED;
      }
      if(isBottomPressed) {
          intakeMoverMax.getEncoder().setPosition(-17.976144790649414);
          currentState = eCurrentState.OUT_STOPPED;
      }
        
      if(currentDesiredState == eDesiredEndState.IN) {
        desiredPositionTarget = -17.976144790649414;
        if (!isTopPressed){
          currentState = eCurrentState.MOVING_OUT;
        }
      } 
      
      if(currentDesiredState == eDesiredEndState.OUT) {
        desiredPositionTarget = 0;
        if (!isBottomPressed){
          currentState = eCurrentState.MOVING_IN;
        }          
      }

      m_controller.setSetpoint(desiredPositionTarget, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    });
  }


  public Command Stop(){
    return run( () -> {
      intakeMoverMax.set(0);
    });
  }


}
