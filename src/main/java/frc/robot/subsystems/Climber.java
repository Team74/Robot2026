package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{

    SparkMax climbMax = new SparkMax(Constants.ClimberConstants.ClimbMotorID, MotorType.kBrushless);
    double climbSpeed = Constants.ClimberConstants.ClimbSpeed;

    public Command Climb(boolean reverse){
      return run(()->{
        double desiredSpeed = climbSpeed;
        if(reverse){
            desiredSpeed = -desiredSpeed;
        }
        climbMax.set(desiredSpeed);
      });
    } 


    public Command ClimbUp(){
        return run(()->{
          climbMax.set(climbSpeed);
          System.out.println("Hi" + climbSpeed);
        });
    } 
    public Command ClimbDown(){
        return run(()->{
          climbMax.set(-climbSpeed);
          System.out.println("down" + climbSpeed);
        });
    } 
    public Command ClimbStop(){
        return run(()->{
                    //System.out.println("stop" + climbSpeed);

          climbMax.set(0);
        });
    } 
}
