package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MatchState {
    private MatchState() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    private static Optional<Alliance> autoVictor = Optional.empty();

    /**
     * Method to determine the winner of the autonomous period, done by accessing
     * the {@link DriverStation}'s game-specific message. This can be used, with the
     * match time, to determine which hub is active.
     * 
     * @return an {@link Optional} of an {@link Alliance}, representing whether or
     *         not we know the winner of auto and, if so, who it was.
     */
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

    /**
     * Whether or not the
     * 
     * @param allianceToCheck an {@link Alliance}
     * @return
     */
    public static boolean didWinAuto(Alliance allianceToCheck) {
        return allianceToCheck.equals(autoVictor.get());
    }
}
