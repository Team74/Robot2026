package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ArcSwerve extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController controller;

    private final SwerveRequest.FieldCentricFacingAngle driveRequest;

    SlewRateLimiter rotLimiter = new SlewRateLimiter(360);

    public ArcSwerve(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentricFacingAngle request, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.driveRequest = request;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        var targetHeading = calcRotation2d(this.drivetrain);
        
        var rot = rotLimiter.calculate(targetHeading.getDegrees());

        var rotDeg = new Rotation2d(rot);

        drivetrain.setControl(driveRequest
            .withVelocityX(MathUtil.applyDeadband(-controller.getLeftY() * Constants.MAX_SPEED, 0.1))
            .withVelocityY(MathUtil.applyDeadband(-controller.getLeftX() * Constants.MAX_SPEED, 0.1))
            .withTargetDirection(rotDeg));
    }

    public static Rotation2d calcRotation2d(CommandSwerveDrivetrain drivetrain) {
        //CurrentPose
        Pose2d robotPose = drivetrain.getState().Pose;

        //ShooterOffset
        Pose2d shooterPose = robotPose.plus(Constants.VisionConstants.shooterRelativeToBot);
        
        //What Alliance
        var alliance = Alliance.Blue;

        if (!DriverStation.getAlliance().isEmpty()){
            alliance = DriverStation.getAlliance().get();
        }
        boolean isBlue = (alliance == Alliance.Blue);

        Translation2d targetPos = isBlue ? Constants.FieldTargets.blueHub : Constants.FieldTargets.redHub;

        Rotation2d targetHeading = targetPos.minus(shooterPose.getTranslation())
            .getAngle()
            .minus(Constants.VisionConstants.shooterRelativeToBot.getRotation());

        if (alliance == Alliance.Red) {
            targetHeading = targetHeading.minus(new Rotation2d(Math.toRadians(180.0)));
        }

        SmartDashboard.putNumber("targetHeading", targetHeading.getDegrees());

        double distance = shooterPose.getTranslation().getDistance(targetPos);

        SmartDashboard.putNumber("DistanceToTarget", distance);

        return targetHeading;
    }
}