package frc.lib.generic.simulation.mechanisms;


import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.List;

import static frc.lib.generic.simulation.mechanisms.MechanismConstants.*;
import static frc.lib.generic.simulation.mechanisms.MechanismUtilities.generateArrowLigaments;

/**
 * A Mechanism2d object to display the current velocity and target velocity of a mechanism.
 */
public class SpeedMechanism2d {
    private final String key;

    private Mechanism2d mechanism;

    private MechanismLigament2d currentVelocityLigament;
    private MechanismLigament2d targetVelocityLigament;
    private MechanismLigament2d targetVelocityTopArrowLigament;
    private MechanismLigament2d targetVelocityBottomArrowLigament;
    private MechanismLigament2d currentVelocityTopArrowLigament;
    private MechanismLigament2d currentVelocityBottomArrowLigament;

    private final double deadband;
    private final boolean inverted;

    public SpeedMechanism2d(String key, double maximumDisplayableVelocity, boolean inverted) {
        this(key, maximumDisplayableVelocity, 0.001, inverted);
    }

    public SpeedMechanism2d(String key, double maximumDisplayableVelocity) {
        this(key, maximumDisplayableVelocity, 0.001, false);
    }

    public SpeedMechanism2d(String key, double maximumDisplayableVelocity, double deadband, boolean inverted) {
        this.deadband = deadband;
        this.key = key;
        this.inverted = inverted;

        initializeMechanism(maximumDisplayableVelocity);
    }

    /**
     * Updates the mechanism's velocity and target velocity and logs the Mechanism2d object.
     *
     * @param velocity       the current velocity
     * @param targetVelocity the target velocity
     */
    public void updateMechanism(double velocity, double targetVelocity) {
        setTargetVelocity(targetVelocity);
        updateMechanism(velocity);
    }

    /**
     * Updates the mechanism's velocity and logs the Mechanism2d object.
     *
     * @param velocity the current velocity
     */
    public void updateMechanism(double velocity) {
        velocity = inverted ? -velocity : velocity;
//todo: fix class
        buildArrowAtLigament(velocity, currentVelocityTopArrowLigament, currentVelocityBottomArrowLigament);

        currentVelocityLigament.setLength(velocity);

        setCurrentLigamentColor(velocityToColor(velocity));

        SmartDashboard.putData("Mechanisms/" + key, mechanism);
    }

    /**
     * Sets the target velocity but doesn't log the Mechanism2d object.
     *
     * @param targetVelocity the target velocity
     */
    public void setTargetVelocity(double targetVelocity) {
        targetVelocity = inverted ? -targetVelocity : targetVelocity;

        buildArrowAtLigament(targetVelocity, targetVelocityTopArrowLigament, targetVelocityBottomArrowLigament);
        targetVelocityLigament.setLength(targetVelocity);
    }

    private void setCurrentLigamentColor(Color8Bit color) {
        currentVelocityLigament.setColor(color);
        currentVelocityTopArrowLigament.setColor(color);
        currentVelocityBottomArrowLigament.setColor(color);
    }

    private Color8Bit velocityToColor(double velocity) {
        if (velocity > deadband)
            return POSITIVE_VELOCITY_COLOUR;

        else if (velocity < -deadband)
            return NEGATIVE_VELOCITY_COLOUR;

        return NO_VELOCITY_COLOUR;
    }

    private void buildArrowAtLigament(double velocity, MechanismLigament2d topWing, MechanismLigament2d bottomWing) {
        if (velocity > deadband) {
            topWing.setAngle(TOP_WING_POINT_ANGLE);
            bottomWing.setAngle(BOTTOM_WING_POINT_ANGLE);
        } else if (velocity < -deadband) {
            topWing.setAngle(TOP_WING_POINT_ANGLE - 180);
            bottomWing.setAngle(BOTTOM_WING_POINT_ANGLE - 180);
        } else {
            topWing.setAngle(TOP_WING_UPWARDS_ANGLE);
            bottomWing.setAngle(BOTTOM_WING_DOWNWARDS_ANGLE);
        }
    }

    private void initializeMechanism(double maximumDisplayableVelocity) {
        this.mechanism = new Mechanism2d(2 * maximumDisplayableVelocity, 2 * maximumDisplayableVelocity);

        MechanismRoot2d root = mechanism.getRoot("VelocityRoot", maximumDisplayableVelocity, maximumDisplayableVelocity);

        this.currentVelocityLigament = root.append(new MechanismLigament2d("CurrentVelocityLigament", 0, 0, MECHANISM_LINE_WIDTH, NO_VELOCITY_COLOUR));
        this.targetVelocityLigament = root.append(new MechanismLigament2d("TargetVelocityLigament", 0, 0, MECHANISM_LINE_WIDTH, TARGET_COLOUR));

        List<MechanismLigament2d> currentLigaments = generateArrowLigaments("CurrentVelocity", NO_VELOCITY_COLOUR, SPEED_ARROW_LENGTH_SCALAR * maximumDisplayableVelocity);

        this.currentVelocityTopArrowLigament = currentVelocityLigament.append(currentLigaments.get(0));
        this.currentVelocityBottomArrowLigament = currentVelocityLigament.append(currentLigaments.get(1));

        List<MechanismLigament2d> targetLigaments = generateArrowLigaments("TargetVelocity", TARGET_COLOUR, SPEED_ARROW_LENGTH_SCALAR * maximumDisplayableVelocity);

        this.targetVelocityTopArrowLigament = targetVelocityLigament.append(targetLigaments.get(0));
        this.targetVelocityBottomArrowLigament = targetVelocityLigament.append(targetLigaments.get(1));
    }
}