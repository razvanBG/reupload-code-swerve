package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveDrivetrain;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveModule;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.util.MathUtils;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "SwerveTunerOpMode")
public class SwerveTunerOpMode extends CommandOpMode {
    public static boolean USE_GAMEPAD = false;
    public static boolean ZEROING = false;
    public static double vx;
    public static double vy;
    public static double omega;

    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    TelemetryData telemetryData = new TelemetryData(telemetry);

    private final Robot robot = Robot.getInstance();
    private CoaxialSwerveDrivetrain swerve; // just for easier reading of code

    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

        swerve = robot.drive.swerve;

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        // Driver controls
        // Reset heading
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> robot.drive.setPose(new Pose2d()))
        );
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        Vector2d[] moduleVelocities = new Vector2d[4];
        // Drive the robot
        if (USE_GAMEPAD) {
            // ALL ROBOT CENTRIC FOR TESTING PURPOSES
            double minSpeed = 0.3; // As a fraction of the max speed of the robot
            double speedMultiplier = minSpeed + (1 - minSpeed) * driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            swerve.updateWithTargetVelocity(
                    new ChassisSpeeds(
                            driver.getLeftX() * Constants.MAX_DRIVE_VELOCITY * speedMultiplier,
                            driver.getLeftY() * Constants.MAX_DRIVE_VELOCITY * speedMultiplier,
                            driver.getRightX()
                    )
            );
        } else if (ZEROING) {
            robot.FRmotor.set(0.2);
            robot.FLmotor.set(0.2);
            robot.BLmotor.set(0.2);
            robot.BRmotor.set(0.2);

            robot.FRswervo.getAbsoluteEncoder().zero(Constants.FR_ENCODER_OFFSET);
            robot.FLswervo.getAbsoluteEncoder().zero(Constants.FL_ENCODER_OFFSET);
            robot.BLswervo.getAbsoluteEncoder().zero(Constants.BL_ENCODER_OFFSET);
            robot.BRswervo.getAbsoluteEncoder().zero(Constants.BR_ENCODER_OFFSET);
        } else {
            swerve.setTargetVelocity(
                    new ChassisSpeeds(vx, vy, omega)
            );
            moduleVelocities = swerve.update();
        }

        for (CoaxialSwerveModule module : swerve.getModules()) {
            module.setSwervoPIDF(Constants.SWERVO_PIDF_COEFFICIENTS);
        }

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryData.addData("getPose", robot.drive.getPose());

        telemetryData.addData("Target Chassis Velocity", swerve.getTargetVelocity());
        telemetryData.addData("FR Module", swerve.getModules()[0].getTargetVelocity() + " | " + swerve.getModules()[0].getPowerTelemetry());
        telemetryData.addData("FL Module", swerve.getModules()[1].getTargetVelocity() + " | " + swerve.getModules()[1].getPowerTelemetry());
        telemetryData.addData("BL Module", swerve.getModules()[2].getTargetVelocity() + " | " + swerve.getModules()[2].getPowerTelemetry());
        telemetryData.addData("BR Module", swerve.getModules()[3].getTargetVelocity() + " | " + swerve.getModules()[3].getPowerTelemetry());

        telemetryData.addData("FR error / flipped", swerve.getModules()[0].getAngleError() + " | " + swerve.getModules()[0].wheelFlipped);
        telemetryData.addData("FL error / flipped", swerve.getModules()[1].getAngleError() + " | " + swerve.getModules()[1].wheelFlipped);
        telemetryData.addData("BL error / flipped", swerve.getModules()[2].getAngleError() + " | " + swerve.getModules()[2].wheelFlipped);
        telemetryData.addData("BR error / flipped", swerve.getModules()[3].getAngleError() + " | " + swerve.getModules()[3].wheelFlipped);

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        super.run();
        robot.pinpoint.update();
        telemetryData.update();
    }

    @Override
    public void end() {
        Constants.END_POSE = robot.drive.getPose();
    }
}