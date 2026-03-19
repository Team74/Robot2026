package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class AimBot extends Command {
    private final CommandSwerveDrivetrain drivetrain;



    public AimBot(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }
    
    @Override
    public void execute() {
        //Add Dashboard Values
        //Figure out what you want to do with it

        //This wil calc both distance and rotation to hub
        ArcSwerve.calcRotation2d(drivetrain);
    }
}
