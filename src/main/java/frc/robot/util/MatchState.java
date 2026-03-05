package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MatchState {
    private MatchState() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    private static Optional<Alliance> autoVictor = Optional.empty();

    public static Optional<Alliance> getAutoVictor() {
        if (autoVictor.isPresent()) {
            return autoVictor;
        }

        String gameData = DriverStation.getGameSpecificMessage();

        switch (gameData.charAt(0)) {
            case 'B':
                autoVictor = Optional.of(Alliance.Blue);
                return autoVictor;
            case 'R':
                autoVictor = Optional.of(Alliance.Red);
                return autoVictor;
            default:
                return autoVictor;
        }
    }

    public static boolean didWeWinAuto(Alliance us){
        return us.equals(autoVictor.get());
    }
}
