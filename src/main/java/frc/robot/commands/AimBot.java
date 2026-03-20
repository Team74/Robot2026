package frc.robot.commands;

import com.reduxrobotics.frames.ShortArrayFrame.ShortArrayToType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class AimBot extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final Hood hood;
    



    public AimBot(CommandSwerveDrivetrain drivetrain, Shooter shooter, Hood hood) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.hood = hood;
        addRequirements(shooter);
        
    }
    
    @Override
    public void execute() {
        //Add Dashboard Values
        //Figure out what you want to do with it
        //double hoodTargetValue = hood.hoodTargetValue;
        var DistanceToTarget = SmartDashboard.getNumber("DistanceToTarget", 0);

        //This wil calc both distance and rotation to hub
        ArcSwerve.calcRotation2d(drivetrain);

        if (DistanceToTarget <= 2.3) {
            if (hood.stringPotValue < 10.2 && hood.stringPotValue > 5){
                hood.hoodTargetValue = 3.304 * Math.pow(1.3, DistanceToTarget);
            }
        } else {
            if (hood.stringPotValue < 10.2 && hood.stringPotValue > 5){
                hood.hoodTargetValue = 3.304 * Math.pow(1.3, DistanceToTarget);
            }
            if (shooter.currentRPS_Shooter < 95){
                Constants.ShooterConstants.shooterDesiredRPS = 25 * DistanceToTarget - 15; 
            }
        }

        /*if (DistanceToTarget <= 1.5621){
            hood.hoodTargetValue = 5.2842;
        } 
        else if (DistanceToTarget > 1.5421 && DistanceToTarget <= 1.9939){
            hood.hoodTargetValue = 5.573;
        }
        else if (DistanceToTarget > 1.9939 && DistanceToTarget <= 2.841){
            hood.hoodTargetValue = 5.841;
        }
        else if (DistanceToTarget > 2.841 && DistanceToTarget <= 2.7432){
            hood.hoodTargetValue = 7.1946;
            //Constants.ShooterConstants.shooterDesiredRPS = Constants.ShooterConstants.shooterDesiredRPS + 10;
        } else {
            hood.hoodTargetValue = 7.3;
        }*/
    }
}
