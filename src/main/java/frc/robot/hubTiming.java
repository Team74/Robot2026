package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import org.photonvision.PhotonUtils;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class hubTiming {
    String hubStart;
    boolean isRedHubActive;
    Color allianceColor;
    Timer timer;
    static int autonSecLength = 20;
    static int telopSecLength = 160;
    Alliance alliance;

    gameSegments currentSegment;

    public hubTiming() {
        timer = new Timer();

        if (DriverStation.getAlliance().isPresent()){
            alliance = DriverStation.getAlliance().get();

            switch (alliance.toString()){
                case "Blue": 
                    allianceColor = Color.kBlue;
                break;

                case "Red": 
                    allianceColor = Color.kRed;
                break;
            }
        }
    }

    public enum gameSegments {
        AUTON(0, 20, "BOTH"),
        TELOPTRANS(20, 30, "BOTH"),
        SHIFT1(30, 55, "L"),
        SHIFT2(55, 80, "W"),
        SHIFT3(80, 105, "L"),
        SHIFT4(105, 130, "W"),
        ENDGAME(130, 160, "BOTH");

        final int startSec;
        final int endSec;
        final String whosActive;

        private gameSegments(int startSec, int endSec, String whosActive) {
            this.startSec = startSec;
            this.endSec = endSec;
            this.whosActive = whosActive;
        }
    }

    public Optional<Time> timeRemainingInShift() {
        return getCurrentShift().map((s) -> Seconds.of(s.endSec - getMatchTime()));
    }

    public Optional<gameSegments> getCurrentShift() {
        double matchTime = getMatchTime();
        if (matchTime < 0) {
            return Optional.empty();
        }

        for (gameSegments s : gameSegments.values()) {
            if (matchTime < s.endSec) {
                currentSegment = s;

                return Optional.of(s);
            }
        }
        return Optional.empty();
    }

    public double getMatchTime() {
        if (DriverStation.isAutonomous()) {
            if (DriverStation.getMatchTime() < 0) {
                return DriverStation.getMatchTime();
            }
            return autonSecLength - DriverStation.getMatchTime();
        } else if (DriverStation.isTeleop()) {
            if (DriverStation.getMatchTime() < 0) {
                return DriverStation.getMatchTime();
            }
            return telopSecLength - DriverStation.getMatchTime();
        }
        return -1;
    }

    public void updateDashboard() {
      SmartDashboard.putBoolean("Hub/Our Hub Active", HubTimer());
      SmartDashboard.putNumber("Hub/Timer", timeRemainingInShift().map(t -> t.in(Units.Seconds)).orElse(0.0));
      SmartDashboard.putString("Hub/Game Seq", currentSegment != null ? currentSegment.toString() : "Not Running");
    }

    boolean didWeWinAuton = false;
    public boolean HubTimer() {
        double matchTime = getMatchTime();
        String msg = DriverStation.getGameSpecificMessage();
        char gameMsg = msg.length() > 0 ? msg.charAt(0) : ' ';

        boolean redInactiveFirst = false;
        switch (gameMsg){
            case 'R': 
                redInactiveFirst = true;
            break;

            case 'B': 
                redInactiveFirst = false;
            break;
        } 

        boolean shift1Active = switch (alliance) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };
        
        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }
}
