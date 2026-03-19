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
import frc.robot.Constants;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfigAccessor;

public class Hood extends SubsystemBase {
    SparkMax hood = new SparkMax(Constants.ShooterConstants.HoodMotorID,MotorType.kBrushed);
    
    //AnalogInput stringPotInput = new AnalogInput(0);
    AnalogPotentiometer stringPot = new AnalogPotentiometer(0, 45, 0);
    
    
    double hoodSpeed = Constants.ShooterConstants.hoodSpeed;
    double stringPotValue = stringPot.get();

    @Override
    public void periodic() {
      
    }

    public Command JumpToTarget(){
          return run(()->{
            var target_Lowest = 3;
        });
    }

    public Command MoveHoodUp(){
        return run(()->{

            if (stringPotValue >= 10.29) {
                hoodSpeed = 0;
            } 

            hood.set(hoodSpeed * -1);
            
        });
    }

    public Command MoveHoodDown(){
        return run(()->{
            if (stringPotValue <= 4.85) {
               hoodSpeed = 0;
            }
            hood.set(hoodSpeed);
        });
    }

    public Command StopHood(){
        return run(()->{
            hood.set(0);
            hoodSpeed = Constants.ShooterConstants.hoodSpeed;
        });
    }

}
