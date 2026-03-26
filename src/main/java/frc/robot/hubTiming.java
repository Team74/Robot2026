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

    gameSegments currentSegment;

    public hubTiming() {
        timer = new Timer();

        if (DriverStation.getAlliance().isPresent()){
            var alliance = DriverStation.getAlliance().get();

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

    public boolean HubTimer() {
        String msg = DriverStation.getGameSpecificMessage();
        char gameMsg = msg.length() > 0 ? msg.charAt(0) : ' ';
        switch (gameMsg){
            case 'R': 
                isRedHubActive = false;
            break;

            case 'B': 
                isRedHubActive = true;
            break;
        } 

        if(currentSegment != null) {
            if(currentSegment.whosActive == "BOTH") {
                return true;  
            }
            //Winner
            if(currentSegment.whosActive == "W" && allianceColor == Color.kRed) {
                return true;  
            }
            //Loser
            if(currentSegment.whosActive == "L" && allianceColor != Color.kRed) {
                return true;  
            }
            else {
                return false;  
            }
        }
        else {
            return false;  
        }
    }
}
