package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{

    SparkMax climberMotor = new SparkMax(Constants.ClimberConstants.ClimbMotorID, MotorType.kBrushless);
    double climbSpeed = Constants.ClimberConstants.ClimbSpeed;

    public Command ClimbUp(){
        return run(()->{
          climberMotor.set(climbSpeed);
        });
    } 
    public Command ClimbDown(){
        return run(()->{
          climberMotor.set(climbSpeed * -1);
        });
    } 
    public Command ClimbStop(){
        return run(()->{
          climberMotor.set(0);
        });
    } 
}
