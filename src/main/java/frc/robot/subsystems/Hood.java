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
    double hoodSpeed = Constants.ShooterConstants.hoodSpeed;
   
    AnalogPotentiometer stringPot = new AnalogPotentiometer(0);
    double stringPotValue = stringPot.get();

    public Command TestStringPotentiometer(){
          return run(()->{
            System.out.println(stringPotValue);
        });
    }

    public Command MoveHoodIn(){
        return run(()->{
            hood.set(hoodSpeed * -1);
        });
    }

    public Command MoveHoodOut(){
        return run(()->{
            hood.set(hoodSpeed);
            // if (magSwitch.get() == false){
            //     hood.set(0);
            //     System.out.println("SWITCHED");
            // }
        });
    }

    public Command StopHood(){
        return run(()->{
            hood.set(0);
        });
    }

}
