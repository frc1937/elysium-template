package frc.lib.generic.mechanisms;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class MechanismConstants {
    protected static final double MECHANISM_WIDTH_RATIO = 0.5;

    protected static final double MECHANISM_LINE_LENGTH = 50;
    protected static final double MECHANISM_LINE_WIDTH = 1;
    protected static final double TARGET_LIGAMENT_WIDTH = 10;

    protected static final String CURRENT_POS = "CurrentPositionLigament";
    protected static final String TARGET_POS = "TargetPositionLigament";

    protected static final Color8Bit TARGET_COLOUR = new Color8Bit(Color.kGray),
            NO_VELOCITY_COLOUR = new Color8Bit(Color.kBlue),
            POSITIVE_VELOCITY_COLOUR = new Color8Bit(Color.kGreen),
            NEGATIVE_VELOCITY_COLOUR = new Color8Bit(Color.kRed);

    protected static final double SPEED_ARROW_LENGTH_SCALAR = 0.2;

    protected static final double
            TOP_WING_POINT_ANGLE = 225,
            BOTTOM_WING_POINT_ANGLE = 135,

            TOP_WING_UPWARDS_ANGLE = 90,
            BOTTOM_WING_DOWNWARDS_ANGLE = 270;

    protected static final double ELEVATOR_STARTING_ANGLE = 0.0;
}
