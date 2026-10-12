package org.firstinspires.ftc.teamcode.globals;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Constants {
    // General
    public enum OpModeType {
        AUTO,
        TELEOP
    }
    public enum AllianceColor {
        BLUE(1), RED(-1);

        private int val;

        AllianceColor(int multiplier) {
            val = multiplier;
        }

        public int getMultiplier() {
            return val;
        }
    }

    public static OpModeType OP_MODE_TYPE;
    public static AllianceColor ALLIANCE_COLOR = AllianceColor.BLUE;
    public static double VOLTAGE_SENSOR_POLLING_RATE = 5; // Hertz
    public static boolean PROBLEMATIC_TELEMETRY = false;

    // Drive
    public static Pose2d END_POSE = new Pose2d();
    public static DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;
    public static AngleUnit ANGLE_UNIT = AngleUnit.RADIANS;

    public static double TRACK_WIDTH = 9.84; // Inches
    public static double WHEEL_BASE = 11.42; // Inches
    public static double MAX_DRIVE_VELOCITY = 83; // Inches/second
    public static double MAX_ANGULAR_VELOCITY = MAX_DRIVE_VELOCITY / Math.hypot(TRACK_WIDTH / 2, WHEEL_BASE / 2);
    public static double PINPOINT_TELEOP_POLLING_RATE = 10; // Hertz
    public static double PINPOINT_AUTO_POLLING_RATE = 100; // Hertz // TODO: needs to be tuned

    public static double STRAFING_SLEW_RATE_LIMIT = 6.7; // Joystick/second
    public static double TURNING_SLEW_RATE_LIMIT = 8.67; // Joystick/second
    public static double JOYSTICK_DEAD_ZONE = 0.02; // Joystick
    public static double MAX_TELEOP_HEADING_CORRECTION_VEL = 1.0; // Radians/second

    public static PIDFCoefficients SWERVO_PIDF_COEFFICIENTS = new PIDFCoefficients(0.4, 0, 0.15, 0.15);
    public static double FR_ENCODER_OFFSET = 2.816; // Radians
    public static double FL_ENCODER_OFFSET = 0.564; // Radians
    public static double BL_ENCODER_OFFSET = 0.287; // Radians
    public static double BR_ENCODER_OFFSET = 2.38189; // Radians

    public static PIDFCoefficients XY_COEFFICIENTS = new PIDFCoefficients(10, 0, 0.8, 0); // Coefficients for inches
    public static PIDFCoefficients HEADING_COEFFICIENTS = new PIDFCoefficients(10, 0, 1, 0); // Coefficients for radians
    public static PIDFCoefficients TELEOP_HEADING_COEFFICIENTS = new PIDFCoefficients(6.7, 0, 0.25, 0); // Coefficients for radians
    public static double XY_TOLERANCE = 0.41; // Inches
    public static double HEADING_TOLERANCE = 0.041; // Radians
    public static double XY_MIN_OUTPUT = 12; // Inches/second
    public static double HEADING_MIN_OUTPUT = 2; // Radians/second

    // Intake

}