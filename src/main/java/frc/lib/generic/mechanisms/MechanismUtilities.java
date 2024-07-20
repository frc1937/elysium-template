package frc.lib.generic.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.ArrayList;
import java.util.List;

import static frc.lib.generic.mechanisms.MechanismConstants.*;

public class MechanismUtilities {

    public static List<MechanismLigament2d> generateLigaments(Color8Bit mechanismColour, double currentLength, double targetLength) {
        List<MechanismLigament2d> ligaments = new ArrayList<>();

        try (MechanismLigament2d currentPose = new MechanismLigament2d(CURRENT_POS, currentLength, 0, MECHANISM_LINE_WIDTH, mechanismColour)) {
            ligaments.add(currentPose);
        }

        try (MechanismLigament2d targetPose = new MechanismLigament2d(TARGET_POS, targetLength, 0, MECHANISM_LINE_WIDTH, TARGET_COLOUR)) {
            ligaments.add(targetPose);
        }

        return ligaments;
    }

    public static List<MechanismLigament2d> generateLigaments(Color8Bit mechanismColour, double length) {
        return generateLigaments(mechanismColour, length, length);
    }

    public static List<MechanismLigament2d> generateArrowLigaments(String name, Color8Bit mechanismColour, double length) {
        List<MechanismLigament2d> ligaments = new ArrayList<>();

        try (MechanismLigament2d topWing = new MechanismLigament2d("TopArrowWing" + name
                , length, TOP_WING_UPWARDS_ANGLE, MECHANISM_LINE_WIDTH, mechanismColour)) {
            ligaments.add(topWing);
        }

        try (MechanismLigament2d bottomWing = new MechanismLigament2d("BottomArrowWing" + name
                , length, BOTTOM_WING_DOWNWARDS_ANGLE, MECHANISM_LINE_WIDTH, mechanismColour)) {
            ligaments.add(bottomWing);
        }

        return ligaments;
    }
}
