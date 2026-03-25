package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
     * Whether or not the specified alliance won auto.
     *
     * If an alliance could not be retrieved, true is returned.
     * 
     * @param allianceToCheck an {@link Alliance}
     */
    public static boolean didWinAuto(Alliance allianceToCheck) {
        return getAutoVictor().orElse(allianceToCheck).equals(allianceToCheck);
    }

    public static boolean didWeWinAuto() {
        var alliance = DriverStation.getAlliance();
        return (DriverStation.getAlliance().isPresent()) ? didWinAuto(alliance.get()) : false;
    }

    public static boolean isHubActive(Optional<Alliance> allianceToCheck) {
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (allianceToCheck.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its
        // likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = didWinAuto(Alliance.Red);

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (allianceToCheck.get()) {
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

    public static boolean ourHubActive() {
        return (DriverStation.getAlliance().isPresent()) ? isHubActive(DriverStation.getAlliance()) : false;
    }

    public static Trigger hubActiveTrigger = new Trigger(MatchState::ourHubActive);
}
