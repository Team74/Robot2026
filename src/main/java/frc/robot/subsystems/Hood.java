package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeFlipper.eDesiredEndState;

import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfigAccessor;

public class Hood extends SubsystemBase {
    SparkMax hood = new SparkMax(Constants.ShooterConstants.HoodMotorID,MotorType.kBrushed);
    
    //AnalogInput stringPotInput = new AnalogInput(0);
    AnalogPotentiometer stringPot = new AnalogPotentiometer(0, 45, 0);
    
    
    double hoodSpeed = Constants.ShooterConstants.hoodSpeed;
    public double stringPotValue = stringPot.get();
    public double hoodTargetValue = 4.99;

    public eCurrentHoodState currentState = eCurrentHoodState.AT_LOWEST; 
    public eDesiredHoodTarget desiredTarget = eDesiredHoodTarget.LOWEST_TARGET; 

    double target_lowest = 5.18;
    double target_low = 6;
    double target_medium = 7;
    double target_high = 8.5;
    double target_highest = 10;
    double hoodCurrentSpeed = hood.get();

    public enum eDesiredHoodTarget {
    LOWEST_TARGET,
    LOW_TARGET,
    MEDIUM_TARGET,
    HIGH_TARGET,
    HIGHEST_TARGET
  }
  public enum eCurrentHoodState {
    AT_LOWEST,
    AT_LOW,
    AT_MEDIUM,
    AT_HIGH,
    AT_HIGHEST,
    MOVING,
    MANUAL
  }

    @Override
    public void periodic() {
        stringPotValue = stringPot.get();
        hoodCurrentSpeed = hood.get();

        if (hoodCurrentSpeed != 0){
            currentState = eCurrentHoodState.MANUAL;
        }

          if(stringPotValue > target_lowest - 0.2 && stringPotValue < target_lowest + 0.2 ){
                currentState = eCurrentHoodState.AT_LOWEST;
            } 
            else if(stringPotValue > target_low - 0.2 && stringPotValue < target_low + 0.2){
                currentState = eCurrentHoodState.AT_LOW;
            }
            else if(stringPotValue > target_medium - 0.2 && stringPotValue < target_medium + 0.2){
                currentState = eCurrentHoodState.AT_MEDIUM;
            }
            else if(stringPotValue > target_high - 0.2 && stringPotValue < target_high + 0.2){
                currentState = eCurrentHoodState.AT_HIGH;
            }
            else if(stringPotValue > target_highest - 0.2 && stringPotValue < target_highest + 0.2){
                currentState = eCurrentHoodState.AT_HIGHEST;
            } else if (hoodCurrentSpeed != 0) {
                currentState = eCurrentHoodState.MOVING;
            } else {
                currentState = eCurrentHoodState.MANUAL;
            }
    }

    public Command SetTarget(){
        return run(()->{
            
            if(desiredTarget == eDesiredHoodTarget.LOWEST_TARGET) {
                desiredTarget = eDesiredHoodTarget.LOW_TARGET;
            }
            else if (desiredTarget == eDesiredHoodTarget.LOW_TARGET) {
                desiredTarget = eDesiredHoodTarget.MEDIUM_TARGET;
            }
            else if (desiredTarget == eDesiredHoodTarget.MEDIUM_TARGET) {
                desiredTarget = eDesiredHoodTarget.HIGH_TARGET;
            }
            else if (desiredTarget == eDesiredHoodTarget.HIGH_TARGET) {
                desiredTarget = eDesiredHoodTarget.HIGHEST_TARGET;
            }
            else if (desiredTarget == eDesiredHoodTarget.HIGHEST_TARGET) {
                desiredTarget = eDesiredHoodTarget.LOWEST_TARGET;
            }

        });
    }

public Command JumpToTarget(){
        return run(()->{

            if(currentState == eCurrentHoodState.MANUAL) {
                hoodTargetValue = target_low;
            }
            if(currentState == eCurrentHoodState.AT_LOWEST && desiredTarget != eDesiredHoodTarget.LOWEST_TARGET) {
                hoodTargetValue = target_low;
            } 
            else if(currentState == eCurrentHoodState.AT_LOW && desiredTarget != eDesiredHoodTarget.LOW_TARGET){
                hoodTargetValue = target_medium;
            } 
            else if(currentState == eCurrentHoodState.AT_MEDIUM && desiredTarget != eDesiredHoodTarget.MEDIUM_TARGET){
                hoodTargetValue = target_high;
            }
            else if(currentState == eCurrentHoodState.AT_HIGH && desiredTarget != eDesiredHoodTarget.HIGH_TARGET){
                hoodTargetValue = target_highest;
            } 
            else if(currentState == eCurrentHoodState.AT_HIGHEST && desiredTarget != eDesiredHoodTarget.HIGHEST_TARGET){
                hoodTargetValue = target_low;
                hoodSpeed = hoodSpeed * -1;
            }

            if (stringPotValue > hoodTargetValue - 0.2 && stringPotValue < hoodTargetValue + 0.2){
                hoodSpeed = 0;
            } else if (stringPotValue > 10.2 || stringPotValue < 5.18){
                hoodSpeed = 0;
            }

            hood.set(hoodSpeed);
           // System.out.println("current state: " + currentState + "desired target: " + desiredTarget);
        });
    }

    public Command Print(){
        return run(()->{
            System.out.println(stringPotValue);
        });
    }

    public Command MoveHoodUp(){
        return run(()->{
            

            if (stringPotValue >= 10.29) {
                hoodSpeed = 0;
            }

            hood.set(hoodSpeed);
          //  System.out.println(stringPotValue);
        });
    }

    public Command MoveHoodDown(){
        return run(()->{
         
            if (stringPotValue <= 5.18) {
               hoodSpeed = 0;
            }
            hood.set(hoodSpeed * -1);
          //  System.out.println(stringPotValue);
        });
    }

    public Command StopHood(){
        return run(()->{
            hood.set(0);
            hoodSpeed = Constants.ShooterConstants.hoodSpeed;
        });
    }

    public Command MoveToSetTarget(){
        return run(()->{
            //hoodSpeed = Constants.ShooterConstants.hoodSpeed;
            if (stringPotValue > hoodTargetValue) {
                hood.set(-hoodSpeed);
            } 
            else if(stringPotValue < hoodTargetValue) {
                hood.set(hoodSpeed);
            } 
            if (stringPotValue > hoodTargetValue -0.5 && stringPotValue < hoodTargetValue + 0.5) {
                hoodSpeed = 0;
                hood.set(hoodSpeed);
            }
            //System.out.println(hoodTargetValue);
        });
    }

}
