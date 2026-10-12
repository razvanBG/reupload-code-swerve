package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveModule;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.commands.DriveTo;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.HoodController;
import org.firstinspires.ftc.teamcode.globals.IntakeController;
import org.firstinspires.ftc.teamcode.globals.RampController;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.teamcode.globals.RobotMap;
import org.firstinspires.ftc.teamcode.globals.TurretController;

import java.util.ArrayList;



@Config
@Autonomous(name = "euler(far auto)", preselectTeleOp = "")
public class eulerFar extends CommandOpMode {

    public static int allianceMultiplier=-1;

    /// ///teleop declarations
    public IntakeController intakeController;
    public RampController rampController;
    public HoodController hoodController;
    public TurretController turretController;

    public static int shoot_far = 1750;
    public static int shoot_top_key = 3150;
    public static int shoot_mid = 1300;
    public static double targetVelocity ,velocity;
    double turret_target_position = 0;

    private double TICKS_TO_DEGREES_SCALING = 0.6752;  // TUNE THIS!
    public static double TURRET_MIN_ANGLE=-178;
    public static double TURRET_MAX_ANGLE=178;

    double minTicks = degreesToEncoderTicks(TURRET_MIN_ANGLE);
    double maxTicks = degreesToEncoderTicks(TURRET_MAX_ANGLE);

    boolean turretOn=false;

    public static double robot_pose_x,robot_pose_y,robot_angle,goal_pose_x=86,goal_pose_y=190;


    private RobotMap map;

    private org.firstinspires.ftc.teamcode.globals.PIDFController controller;
    private DcMotorEx motor;
    private DcMotorEx motor1;
    public static double P=0.005,I=0.03,kV=0.00032,kS=0.05,kD=0;


    /// ///////AUTO DECLARATIONS
    public ElapsedTime timer;

    TelemetryData telemetryData = new TelemetryData(
            new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
    );

    private final Robot robot = Robot.getInstance();

    public static int time1=3000;
    public static int time2=2000;
    public static int time3=1000;
    public static int time4=1000;
    public static int time5=2500;
    public static int time6=1800;
    public static int time7=1000;
    public static int time8=3000;



    public ArrayList<Pose2d> pathPoses;

    public void generatePath() {
        pathPoses = new ArrayList<>();
        pathPoses.add(new Pose2d(0, 0, Math.toRadians(0))); // Starting Pose
        pathPoses.add(new Pose2d(81.266569959659485, -9.31*allianceMultiplier, Math.toRadians(0))); // Line 1
        pathPoses.add(new Pose2d(0, -10*allianceMultiplier, Math.toRadians(0))); // Line 1
        pathPoses.add(new Pose2d(36.48, -41.25*allianceMultiplier, Math.toRadians(0))); // Line 1
        pathPoses.add(new Pose2d(65, -40*allianceMultiplier, Math.toRadians(0))); // Line 1
        pathPoses.add(new Pose2d(0, -10*allianceMultiplier, Math.toRadians(0))); // Line 1
        pathPoses.add(new Pose2d(30.48, -80*allianceMultiplier, Math.toRadians(0))); // Line 1
        pathPoses.add(new Pose2d(65, -80*allianceMultiplier, Math.toRadians(0))); // Line 1
        pathPoses.add(new Pose2d(0, -10*allianceMultiplier, Math.toRadians(0))); // Line 1
        pathPoses.add(new Pose2d(30, -10*allianceMultiplier, Math.toRadians(0))); // Line 1








    }

    @Override
    public void initialize() {
        generatePath();

        if(ALLIANCE_COLOR==AllianceColor.BLUE)
            allianceMultiplier=1;
        else allianceMultiplier=-1;

        /// ///teleop stuff
        motor = hardwareMap.get(DcMotorEx.class, "flywheelMotorR");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1 = hardwareMap.get(DcMotorEx.class, "flywheelMotorL");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        map = new RobotMap(hardwareMap);


// Controllers (non-command based)
        intakeController = new IntakeController(map);
        rampController   = new RampController(map);
        hoodController   = new HoodController(map);
        turretController   = new TurretController(map);

        intakeController.update();
        rampController.update();
        hoodController.update();
        turretController.update(0,turret_target_position);


/// ////// auto normal
        controller = new org.firstinspires.ftc.teamcode.globals.PIDFController(P,I,kD, 0.0);
        timer = new ElapsedTime();

        // Must have for all opModes
        OP_MODE_TYPE = OpModeType.AUTO;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);



        // Schedule the full auto
        robot.drive.setPose(pathPoses.get(0));
        schedule(
                new SequentialCommandGroup(
                        /*shootFirst(),
                        new InstantCommand(()->intakeController.currentStatus= IntakeController.IntakeStatus.COLLECT),
                        new DriveTo(pathPoses.get(1)).withTimeout(3000),
                        new InstantCommand(()->intakeController.currentStatus= IntakeController.IntakeStatus.OFF),
                        new DriveTo(pathPoses.get(2)).withTimeout(2000),
                        shootSequence(),
                        new InstantCommand(()->intakeController.currentStatus= IntakeController.IntakeStatus.COLLECT),
                        new DriveTo(pathPoses.get(1)).withTimeout(3000),
                        new InstantCommand(()->intakeController.currentStatus= IntakeController.IntakeStatus.OFF),
                        new DriveTo(pathPoses.get(2)).withTimeout(2000),
                        *//*new DriveTo(pathPoses.get(3)).withTimeout(time3),
                        new InstantCommand(()->intakeController.currentStatus= IntakeController.IntakeStatus.COLLECT),
                        new DriveTo(pathPoses.get(4),0.8).withTimeout(time4),
                        new InstantCommand(()->intakeController.currentStatus= IntakeController.IntakeStatus.OFF),
                        new DriveTo(pathPoses.get(5)).withTimeout(time5),*//*
                        shootSequence(),
                        new InstantCommand(()->intakeController.currentStatus= IntakeController.IntakeStatus.COLLECT),
                        new DriveTo(pathPoses.get(1)).withTimeout(3000),
                        new InstantCommand(()->intakeController.currentStatus= IntakeController.IntakeStatus.OFF),
                        new DriveTo(pathPoses.get(2)).withTimeout(2000),
                        shootSequence(),*/
                        new DriveTo(pathPoses.get(9)),
                        new InstantCommand(()->turretOn=false),
                        new WaitCommand(1000000)









                        )
        );
    }

    @Override
    public void initialize_loop() {
        telemetryData.addData("ALLIANCE_COLOR", ALLIANCE_COLOR);
        telemetryData.update();
    }

    @Override
    public void run() {
        // DO NOT REMOVE
        robot.updateLoop(telemetryData);

/// //////teleop
        robot_pose_x=robot.drive.getPose().getX();
        robot_pose_y=robot.drive.getPose().getY();
        robot_angle=robot.drive.getPose().getHeading();

        double bearingToGoal = Math.atan2(
                goal_pose_y - robot_pose_y,
                goal_pose_x - robot_pose_x
        );


        double turretAngle = getTurretAngleRadians();  // radians, robot-relative

        // Step 4: Calculate how much turret needs to turn
        // The turret's ABSOLUTE field angle = turretAngle + robotHeading
        // We want: bearingToGoal = turretAngle + robotHeading + angleToTurn
        // Therefore: angleToTurn = bearingToGoal - (turretAngle + robotHeading)
        double angleToTurn = bearingToGoal - (turretAngle + robot_angle);

        angleToTurn = normalizeAngle(angleToTurn);

        double degreesToTurn = Math.toDegrees(angleToTurn);

        if(turretOn)turretController.currentStatus= TurretController.TurretStatus.RUNTO;
        if(!turretOn)turretController.currentStatus= TurretController.TurretStatus.INIT;

        turret_target_position=turnTurretByDegrees(degreesToTurn);

        controller.setPIDF(P,I, kD, kV * targetVelocity + kS);
        velocity = motor1.getVelocity();
        motor.setPower(controller.calculate(targetVelocity - velocity));
        motor1.setPower(controller.calculate(targetVelocity - velocity));

        /// ///////
        // Update any constants that are being updated by FTCDash - used for tuning
        for (CoaxialSwerveModule module : robot.drive.swerve.getModules()) {
            module.setSwervoPIDF(SWERVO_PIDF_COEFFICIENTS);
        }
        ((PIDFController) robot.drive.follower.xController).setCoefficients(XY_COEFFICIENTS);
        ((PIDFController) robot.drive.follower.yController).setCoefficients(XY_COEFFICIENTS);
        ((PIDFController) robot.drive.follower.headingController).setCoefficients(HEADING_COEFFICIENTS);

/// ///teleop
        intakeController.update();
        rampController.update();
        hoodController.update();
        turretController.update(turretController.turret.getCurrentPosition(),turret_target_position);
        /// ////
        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();


        if (PROBLEMATIC_TELEMETRY) {
            robot.profiler.start("High TelemetryData");

            telemetryData.addData("Heading", robot.drive.getPose().getHeading());
            telemetryData.addData("Robot Pose", robot.drive.getPose());telemetryData.addData("FR Module", robot.drive.swerve.getModules()[0].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[0].getPowerTelemetry());
            telemetryData.addData("FL Module", robot.drive.swerve.getModules()[1].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[1].getPowerTelemetry());
            telemetryData.addData("BL Module", robot.drive.swerve.getModules()[2].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[2].getPowerTelemetry());
            telemetryData.addData("BR Module", robot.drive.swerve.getModules()[3].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[3].getPowerTelemetry());

            robot.profiler.end("High TelemetryData");
        }

        robot.profiler.start("Low TelemetryData");
        telemetryData.addData("Robot Target", robot.drive.follower.getTarget());
        telemetryData.addData("atTarget", robot.drive.follower.atTarget());
        telemetryData.addData("X Error", robot.drive.follower.getError().getTranslation().getX());
        telemetryData.addData("Y Error", robot.drive.follower.getError().getTranslation().getY());
        telemetryData.addData("Heading Error", robot.drive.follower.getError().getRotation().getAngle(AngleUnit.RADIANS));



        telemetryData.addData("Target Chassis Velocity", robot.drive.swerve.getTargetVelocity());

        telemetryData.addData("Sigma", "Polar");

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        telemetryData.update();
    }

    @Override
    public void end() {
        END_POSE = robot.drive.getPose();
    }

    /// //commands
    public SequentialCommandGroup shootSequence() {
        return new SequentialCommandGroup(
                new InstantCommand(()->intakeController.currentStatus= IntakeController.IntakeStatus.TRANSFER),
                new InstantCommand(()->turretOn=true),
                new InstantCommand(() ->targetVelocity=shoot_far),
                new InstantCommand(() ->hoodController.currentStatus= HoodController.HoodStatus.FAR),
                new WaitCommand(800),
                new InstantCommand(() ->rampController.currentStatus= RampController.RampStatus.SHOOT),
                new WaitCommand(1000),
                new InstantCommand(() ->targetVelocity=0),
                new InstantCommand(()->intakeController.currentStatus= IntakeController.IntakeStatus.OFF),
                new InstantCommand(()->turretOn=false),
                new InstantCommand(() ->hoodController.currentStatus= HoodController.HoodStatus.INIT),
                new InstantCommand(() ->rampController.currentStatus= RampController.RampStatus.NOSHOOT)
        );
    }
    public SequentialCommandGroup shootFirst() {
        return new SequentialCommandGroup(
                new InstantCommand(()->intakeController.currentStatus= IntakeController.IntakeStatus.TRANSFER),
                new InstantCommand(()->turretOn=true),
                new InstantCommand(() ->targetVelocity=shoot_far+50),
                new InstantCommand(() ->hoodController.currentStatus= HoodController.HoodStatus.FAR),
                new WaitCommand(1000),
                new InstantCommand(() ->rampController.currentStatus= RampController.RampStatus.SHOOT),
                new WaitCommand(1000),
                new InstantCommand(() ->targetVelocity=0),
                new InstantCommand(()->intakeController.currentStatus= IntakeController.IntakeStatus.OFF),
                new InstantCommand(()->turretOn=false),
                new InstantCommand(() ->hoodController.currentStatus= HoodController.HoodStatus.INIT),
                new InstantCommand(() ->rampController.currentStatus= RampController.RampStatus.NOSHOOT)
        );
    }








    public double turnTurretByDegrees(double degrees) {
        // Get current position and calculate target
        double currentTicks = turretController.turret.getCurrentPosition();
        double targetTicks = currentTicks + degreesToEncoderTicks(degrees);


        // Check soft limits
        double targetAngle = getTurretAngleDegrees() + degrees;


        if(targetTicks>maxTicks)targetTicks-=maxTicks;
        if(targetTicks<minTicks)targetTicks-=minTicks;
        return targetTicks;
    }
    // ========== CONVERSION FUNCTIONS ==========.
    public double encoderTicksToDegrees(double ticks) {
        return ticks * TICKS_TO_DEGREES_SCALING;
    }

    public double degreesToEncoderTicks(double degrees) {
        return (degrees / TICKS_TO_DEGREES_SCALING);
    }

    public double getTurretAngleDegrees() {
        return encoderTicksToDegrees(turretController.turret.getCurrentPosition());
    }

    public double getTurretAngleRadians() {
        return Math.toRadians(getTurretAngleDegrees());
    }

    // Normalize angle to [-PI, PI]
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

}