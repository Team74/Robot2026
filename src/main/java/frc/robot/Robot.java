// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ArcSwerve;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    private final boolean kUseLimelight = true;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        m_robotContainer.addTrajectory();

        if (kUseLimelight) {
            var driveState = m_robotContainer.drivetrain.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

            LimelightHelpers.SetRobotOrientation(Constants.VisionConstants.limelightName, headingDeg, 0, 0, 0, 0, 0);
            
            LimelightHelpers.setCameraPose_RobotSpace(Constants.VisionConstants.limelightName,
                -0.32385,    // Forward offset (meters) or y (+y is forward)
                -0.2159,    // Side offset (meters) or x (+x is right)
                0.2492,    // Height offset (meters) or z (+z is up)
                0.0,    // Roll (degrees)
                25.0,   // Pitch (degrees)
                180     // Yaw (degrees)
            );
            
            var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VisionConstants.limelightName);
            if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
            }
        }
    }

    // -8.5 x (in) -> -0.2159 (m)
    // -12.75 y (in) -> -0.32385 (m)
    //  9.8125 z (in) -> 0.2492 (m)

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    // DigitalInput m_toplimitswitch = new DigitalInput(1);
    // DigitalInput m_bottomlimitswitch = new DigitalInput(0);


    @Override
    public void testPeriodic() {
    //   var isTopPressed = m_toplimitswitch.get();
    //   var isBottomPressed = m_bottomlimitswitch.get();
      
    //   System.out.println("isTopPressed: " + isTopPressed + " isBottomPressed: " + isBottomPressed);
    }

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
