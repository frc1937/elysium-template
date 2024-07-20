package frc.lib.generic.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.List;

import static frc.lib.generic.mechanisms.MechanismUtilities.generateLigaments;

public class ElevatorMechanism2d {
    private final String key;

    private final Mechanism2d mechanism;
    private final MechanismLigament2d
            currentPositionLigament,
            targetPositionLigament;

    private final double minimumLength;

    /**
     * Constructs an ElevatorMechanism2d object.
     *
     * @param key            the key of the mechanism
     * @param minimumLength  the minimum length of the elevator
     * @param maximumLength  the maximum length of the elevator
     * @param mechanismColor the color of the mechanism
     */
    public ElevatorMechanism2d(String key, double minimumLength, double maximumLength, Color8Bit mechanismColor) {
        this.key = key;

        this.minimumLength = minimumLength;
        this.mechanism = new Mechanism2d(maximumLength, maximumLength);

        MechanismRoot2d currentPositionRoot = mechanism.getRoot("ZCurrentPositionRoot", 0.5 * maximumLength, 0);
        MechanismRoot2d targetPositionRoot = mechanism.getRoot("TargetPositionRoot", 0.5 * maximumLength, 0);

        List<MechanismLigament2d> ligaments = MechanismUtilities.generateLigaments(mechanismColor, minimumLength, maximumLength);

        this.currentPositionLigament = currentPositionRoot.append(ligaments.get(0));
        this.targetPositionLigament = targetPositionRoot.append(ligaments.get(1));
    }

    /**
     * Updates the mechanism's position and target position, then logs the Mechanism2d object.
     *
     * @param currentPosition the current position
     * @param targetPosition  the target position
     */
    public void updateMechanism(double currentPosition, double targetPosition) {
        setTargetPosition(targetPosition);
        updateMechanism(currentPosition);
    }

    /**
     * Updates the mechanism's position, then logs the Mechanism2d object.
     *
     * @param currentPosition the current position
     */
    public void updateMechanism(double currentPosition) {
        currentPositionLigament.setLength(currentPosition + minimumLength);
        SmartDashboard.putData("Mechanism/" + key, mechanism);
    }

    /**
     * Sets the target position of the mechanism, but doesn't log the Mechanism2d object.
     *
     * @param targetPosition the target position
     */
    public void setTargetPosition(double targetPosition) {
        targetPositionLigament.setLength(targetPosition + minimumLength);
    }
}
