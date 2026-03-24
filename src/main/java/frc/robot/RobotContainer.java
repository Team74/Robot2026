// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AimBot;
import frc.robot.commands.ArcSwerve;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeFlipper;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeFlipper.eCurrentState;
import frc.robot.subsystems.IntakeFlipper.eDesiredEndState;

public class RobotContainer {
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Hood hood = new Hood();
    private final IntakeFlipper intakeFlipper = new IntakeFlipper();
    private final Shooter shooter = new Shooter();
    private final Climber climber = new Climber();
    private final Intake intake = new Intake(shooter);
    private final LEDs led = new LEDs();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0)
      .withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
                                                                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle drivefaceAngle = new SwerveRequest.FieldCentricFacingAngle()
                                                                          .withDeadband(0)
                                                                          .withRotationalDeadband(0)
                                                                          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


    private final Telemetry logger = new Telemetry(Constants.MAX_SPEED, drivetrain.getPigeon2());

    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController operatorXbox = new CommandXboxController(1);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    

    public RobotContainer() {
      drivefaceAngle.HeadingController.setPID(Constants.VisionConstants.arcKp, Constants.VisionConstants.arcKi, Constants.VisionConstants.arcKd);
      drivefaceAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    
       NamedCommands.registerCommand("stopIntake", intake.intakeStop());
      NamedCommands.registerCommand( "SetDesiredStateIn", intakeFlipper.IntakeIn()
                                                            .andThen(intakeFlipper.MoveToDesiredState()));
      NamedCommands.registerCommand("SetDesiredStateOut", intakeFlipper.IntakeOut()
                                                            .andThen(intakeFlipper.MoveToDesiredState()));

      NamedCommands.registerCommand("shoot", shooter.shoot());
      NamedCommands.registerCommand("shootStop", shooter.stopShooter());
        // swaps state then moves until desired state is reached
        NamedCommands.registerCommand("intake Swap", intakeFlipper.SwapDesiredState()
                                                                  .andThen(intakeFlipper.MoveToDesiredState()
                                                                  .until(() -> intakeFlipper.currentState == eCurrentState.OUT_STOPPED && intakeFlipper.currentDesiredState == eDesiredEndState.OUT 
                                                                              || intakeFlipper.currentState == eCurrentState.IN_STOPPED && intakeFlipper.currentDesiredState == eDesiredEndState.IN)));
        NamedCommands.registerCommand("intake", intake.intakeIn());
        NamedCommands.registerCommand("stopShoot", shooter.stopShooter());
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
      DriverStation.silenceJoystickConnectionWarning(true);
     // addTrajectory();
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(MathUtil.applyDeadband(-driverXbox.getLeftY() * Constants.MAX_SPEED, 0.1)) // Drive forward with negative Y (forward)
                    .withVelocityY(MathUtil.applyDeadband(-driverXbox.getLeftX() * Constants.MAX_SPEED, 0.1)) // Drive left with negative X (left)
                    .withRotationalRate(MathUtil.applyDeadband(-driverXbox.getRightX() * Constants.MaxSystemAngularRate, 0.1)) // Drive counterclockwise with negative X (left)
            )     
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        if (DriverStation.isTest() == true) {
          testControls();
        } else {
          controlMapping();
        }

        drivetrain.registerTelemetry(logger::telemeterize);

        SmartDashboard.putNumber("ShooterRPS", shooter.currentRPS_Shooter);

    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    public void addTrajectory() {
      var targetHeading = ArcSwerve.calcRotation2d(drivetrain);

      // var m_trajectory = TrajectoryGenerator.generateTrajectory(
      //        drivetrain.getState().Pose,   
      //        null,   
      //        new Pose2d(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(), targetHeading),
      //        new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));
   
      // logger.TargetTrajectory = m_trajectory;
     
   }

    void controlMapping(){
      //At this point, this will just send data to the dashboard.
      shooter.setDefaultCommand(new AimBot(drivetrain, shooter, hood));
      
      //SmartDashboard.putBoolean("Do I Shoot?", led.HubTimer());

      //DRIVER CONTROLS
      //
      //ROBOT RELATIVE
        driverXbox.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driverXbox.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        driverXbox.povRight().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityY(-0.5).withVelocityX(0))
        );
        driverXbox.povLeft().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityY(0.5).withVelocityX(0))
        );

      //GYRO RESET 
        driverXbox.y()
          .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

      //LOCK DRIVEBASE
    //    driverXbox.a()
    //      .whileTrue(drivetrain.applyRequest(() -> brake));

      //SLOW MODE
      driverXbox.leftBumper().onTrue(runOnce(() -> Constants.MAX_SPEED = Constants.MaxSystemSpeed * Constants.SlowModeDriveMultiplier)
                                    .andThen(() -> Constants.MaxAngularRate = Constants.MaxSystemAngularRate * Constants.SlowModeAngleMultiplier));
      
      driverXbox.leftBumper().onFalse(runOnce(() -> Constants.MAX_SPEED = Constants.MaxSystemSpeed)
                                     .andThen(() -> Constants.MaxAngularRate = Constants.MaxSystemAngularRate));
      
      //APRIL TAG ALIGN
      driverXbox.a().whileTrue(new ArcSwerve(drivetrain, drivefaceAngle, driverXbox));

      //PATHPLANNER ON THE FLY
      var alliance = Alliance.Blue;

      if (DriverStation.getAlliance().isPresent()){
          alliance = DriverStation.getAlliance().get();
      }
      boolean isBlue = (alliance == Alliance.Blue);

      Pose2d targetPose = isBlue ? Constants.VisionConstants.blueShooter : Constants.VisionConstants.redShooter;

      driverXbox.x().whileTrue(drivetrain.path_find_to(targetPose,MetersPerSecond.of(0)));
    
      Trigger reverseIntakeButton = new Trigger(operatorXbox.leftTrigger().and(operatorXbox.b()));
      Trigger reverseShootButton = new Trigger(operatorXbox.rightTrigger().and(operatorXbox.b()));
      Trigger hoodJumpToTargetButton = new Trigger(operatorXbox.rightBumper().and(operatorXbox.rightTrigger()));
      Trigger testJumpToTargetButton = new Trigger(operatorXbox.leftBumper().and(operatorXbox.leftTrigger()));

      hoodJumpToTargetButton
        .onTrue(hood.SetTarget())
        .whileFalse(hood.StopHood());

      operatorXbox.a().onTrue(hood.MoveToSetTarget()).onFalse(hood.StopHood());
      operatorXbox.y().onTrue(hood.Print()).onFalse(hood.StopHood());

      testJumpToTargetButton
        .onTrue(hood.JumpToTarget())
        .whileFalse(hood.StopHood());
      //INTAKE
      operatorXbox.leftTrigger()
        .whileTrue(intake.intakeIn())
        .whileFalse(intake.intakeStop());
      //OUTTAKE
      reverseIntakeButton
        .whileTrue(intake.intakeOut())
        .whileFalse(intake.intakeStop());
      //FLIP INTAKE
      operatorXbox.x().debounce(0.009)
        .onTrue(intakeFlipper.SwapDesiredState())
        .whileFalse(intakeFlipper.MoveToDesiredState());

      //SHOOT
      operatorXbox.rightTrigger()
        .whileTrue(shooter.shoot())
        .whileFalse(shooter.stopShooter());
      //REVERSE SHOOT
      reverseShootButton
        .onTrue(shooter.reverseShoot())
        .whileFalse(shooter.stopShooter());

      //HOOD UP
      operatorXbox.rightBumper()
        .whileTrue(hood.MoveHoodUp())
        .whileFalse(hood.StopHood());
      //HOOD DOWN
      operatorXbox.leftBumper()
        .whileTrue(hood.MoveHoodDown())
        .whileFalse(hood.StopHood());

      //CLIMB UP
      operatorXbox.povUp()
        .onTrue(climber.ClimbUp())
        .whileFalse(climber.ClimbStop());
      //CLIMB DOWN
      operatorXbox.povDown()
        .onTrue(climber.ClimbDown())
        .whileFalse(climber.ClimbStop());
    }


    void testControls() {
      //Shooter Motors
      operatorXbox.rightTrigger()
        .onTrue(shooter.shoot())
        .onFalse(shooter.stopShooter());
      
      //Tower Motor
      operatorXbox.leftTrigger()
        .onTrue(shooter.testTower(operatorXbox.b().getAsBoolean()))
        .onFalse(shooter.stopShooter());

      //Hot dog Motor
      operatorXbox.rightBumper()
        .onTrue(intake.hotdogTest())
        .onFalse(intake.intakeStop());

      //Hood Motor
      operatorXbox.leftBumper()
        .onTrue(hood.MoveHoodUp())
        .onFalse(hood.StopHood());

      //Intake Roller Motor
      operatorXbox.y()
        .onTrue(intake.intakeIn())
        .onFalse(intake.intakeStop());

      //Intake Flipper Motor
      operatorXbox.x()
        .onTrue(intakeFlipper.SwapDesiredState())
        .onFalse(intakeFlipper.MoveToDesiredState());                            

      //Climber Motor
      operatorXbox.a()
        .onTrue(climber.ClimbUp())
        .onFalse(climber.ClimbStop());
    }
}
