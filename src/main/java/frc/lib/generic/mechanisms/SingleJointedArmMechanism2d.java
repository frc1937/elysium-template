package frc.lib.generic.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.List;

import static frc.lib.generic.mechanisms.MechanismConstants.MECHANISM_LINE_LENGTH;
import static frc.lib.generic.mechanisms.MechanismUtilities.generateLigaments;

/**
 * A Mechanism2d object to display the current angle and the target angle of a single jointed arm.
 */
public class SingleJointedArmMechanism2d {
    private final String key;

    private final Mechanism2d mechanism;
    private final MechanismLigament2d
            currentPositionLigament,
            targetPositionLigament;

    public SingleJointedArmMechanism2d(String key, Color8Bit mechanismColor) {
        this.key = key;

        this.mechanism = new Mechanism2d(2 * MECHANISM_LINE_LENGTH,
                2 * MECHANISM_LINE_LENGTH);

        MechanismRoot2d root = mechanism.getRoot(
                "AngleRoot",
                MECHANISM_LINE_LENGTH,
                MECHANISM_LINE_LENGTH
        );

        List<MechanismLigament2d> ligaments = MechanismUtilities.generateLigaments(mechanismColor, MECHANISM_LINE_LENGTH);

        this.currentPositionLigament = root.append(ligaments.get(0));
        this.targetPositionLigament = root.append(ligaments.get(1));
    }

    /**
     * Updates the mechanism's angle and target angle, then logs the Mechanism2d object.
     *
     * @param currentAngle the current angle
     * @param targetAngle  the target angle
     */
    public void updateMechanism(Rotation2d currentAngle, Rotation2d targetAngle) {
        setTargetAngle(targetAngle);
        updateMechanism(currentAngle);
    }

    /**
     * Updates the mechanism's angle, then logs the Mechanism2d object.
     *
     * @param currentAngle the current angle
     */
    public void updateMechanism(Rotation2d currentAngle) {
        currentPositionLigament.setAngle(currentAngle);
        SmartDashboard.putData("Mechanisms/" + key, mechanism);
    }

    /**
     * Sets the target angle of the mechanism.
     *
     * @param targetAngle the target angle
     */
    public void setTargetAngle(Rotation2d targetAngle) {
        targetPositionLigament.setAngle(targetAngle);
    }
}
