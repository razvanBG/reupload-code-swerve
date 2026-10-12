package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.globals.Constants.ALLIANCE_COLOR;
import static org.firstinspires.ftc.teamcode.globals.Constants.END_POSE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveModule;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.HoodController;
import org.firstinspires.ftc.teamcode.globals.IntakeController;
import org.firstinspires.ftc.teamcode.globals.PIDFController;
import org.firstinspires.ftc.teamcode.globals.RampController;
import org.firstinspires.ftc.teamcode.globals.Robot;
import org.firstinspires.ftc.teamcode.globals.RobotMap;
import org.firstinspires.ftc.teamcode.globals.TurretController;

@Config
@TeleOp(name = "florin salam cu finul lui si londonezu", group = "Linear OpMode")
public class SwerveOpMode extends LinearOpMode {

    private PIDFController controller;
    private DcMotorEx motor;
    private DcMotorEx motor1;

    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();
    private RobotMap map;

    private double TICKS_TO_DEGREES_SCALING = 0.6752;  // TUNE THIS!
    public static double TURRET_MIN_ANGLE=-178;
    public static double TURRET_MAX_ANGLE=178;

    double minTicks = degreesToEncoderTicks(TURRET_MIN_ANGLE);
    double maxTicks = degreesToEncoderTicks(TURRET_MAX_ANGLE);

    public static double turretOffset;


    public IntakeController intakeController;
    public RampController rampController;
    public HoodController hoodController;
    public TurretController turretController;

    public static int shoot_far = 1730;
    public static int shoot_top_key = 3150;
    public static int shoot_mid = 1300;


    public static double hood_target_pos = 0.5;
    double turret_target_position = 0;

    public static double targetVelocity,velocity;
    public static double P=0.005,I=0.03,kV=0.00032,kS=0.05,kD=0;

    public static double delayShoot = 0.15;


    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime shootTime = new ElapsedTime();
    private final ElapsedTime shootEndTime = new ElapsedTime();
    private final ElapsedTime shootTransferTime = new ElapsedTime();

    public static double allianceMultiplier;


    boolean flywheelOn=false;
    int noTurret=1;
    boolean flywheelEnd=false;
    boolean shootTransfer=false;
    boolean turretOn=false;
    boolean xDrive=false;


    public static double robot_pose_x,robot_pose_y,robot_angle,goal_pose_x=86,goal_pose_y=-190;
    //VALORI RED FAR
    //goal_pose_x=100.83,goal_pose_y=193.64;

    //VALORI BLUE FAR
    //goal_pose_x=92.87,goal_pose_y=-208.6;

    //VAL FAR NOI blue  x=86,y=-190
    //VALORI BLUE CLOSE
    //foal_pose_x=54,goal_pose_y=-10.81
    private Telemetry dashTelemetry;





    @Override
    public void runOpMode() throws InterruptedException {

        dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        if(ALLIANCE_COLOR== Constants.AllianceColor.BLUE)
            allianceMultiplier=1;
        else allianceMultiplier=-1;
        goal_pose_x=goal_pose_x*allianceMultiplier;

        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;

        // Resets the command scheduler-/+

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

        driver = new GamepadEx(gamepad1);

        // Driver controls
        // Reset heading
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> robot.drive.setPose(new Pose2d()))
        );

        robot.drive.setPose(END_POSE);


        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;

        // === "CommandOpMode thing" (scheduler lifecycle) ===
        // Equivalent to CommandOpMode.super.reset() at init-time
        CommandScheduler.getInstance().reset();

        motor = hardwareMap.get(DcMotorEx.class, "flywheelMotorR");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1 = hardwareMap.get(DcMotorEx.class, "flywheelMotorL");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);


        controller = new PIDFController(P,I,kD, 0.0);


        // Hardware map wrapper
        map = new RobotMap(hardwareMap);

        // Initialize robot + subsystems registration (your Robot likely does this)
        robot.init(hardwareMap);

        // Controllers (non-command based)
        intakeController = new IntakeController(map);
        rampController   = new RampController(map);
        hoodController   = new HoodController(map);
        turretController   = new TurretController(map);



        // Initial updates
        intakeController.update();
        rampController.update();
        hoodController.update();
        turretController.update(0,turret_target_position);

        telemetry.addData("ENDPOSE",END_POSE);

        telemetry.addLine("Dual Flywheel Ready!");
        telemetry.update();

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> robot.drive.setPose(new Pose2d()))
        );
        waitForStart();

        // Once after start (same intent as old run() first-iteration check)
        robot.initHasMovement();
        loopTimer.reset();

        while (opModeIsActive()) {
            if (isStopRequested()) break;
            if (timer == null) {
                END_POSE = new Pose2d(END_POSE.getX(), END_POSE.getY(), END_POSE.getHeading());
                robot.drive.setPose(END_POSE);
                robot.initHasMovement();
                timer = new ElapsedTime();
            }


            for (CoaxialSwerveModule module : robot.drive.swerve.getModules()) {
                module.setSwervoPIDF(Constants.SWERVO_PIDF_COEFFICIENTS);
            }
            double minSpeed = 1.0;
            double speedMultiplier = minSpeed - 0.5 * gamepad1.right_trigger;
            robot.drive.swerve.updateWithTargetVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            driver.getLeftY() * Constants.MAX_DRIVE_VELOCITY * speedMultiplier,
                            -driver.getLeftX() * Constants.MAX_DRIVE_VELOCITY * speedMultiplier,
                            -driver.getRightX() * Constants.MAX_ANGULAR_VELOCITY * speedMultiplier,
                            robot.drive.getPose().getRotation()
                    )
            );
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

            turretController.currentStatus= TurretController.TurretStatus.INIT;

            turret_target_position=turnTurretByDegrees(degreesToTurn);



            controller.setPIDF(P,I, kD, kV * targetVelocity + kS);
            velocity = motor1.getVelocity();
            motor.setPower(controller.calculate(targetVelocity - velocity));
            motor1.setPower(controller.calculate(targetVelocity - velocity));

            // Edge-detection gamepad copies
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if(xDrive)
            {

            }
            // Reset heading (DPAD_UP press)
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                robot.drive.setPose(new Pose2d());
            }

            // Drive




         /*   if (gamepad2.right_bumper) {
                flywheel.setTargetVelocity(x); // RPM
                rampController.currentStatus = RampController.RampStatus.SHOOT;
                intakeController.currentStatus = IntakeController.IntakeStatus.COLLECT;
            }
            if (gamepad2.left_bumper) {
                flywheel.stop();36
                rampController.currentStatus = RampController.RampStatus.NOSHOOT;
            }
*/
            if(currentGamepad2.right_stick_button!=previousGamepad2.right_stick_button)
            {
                switch (noTurret)
                {
                    case 1:
                        noTurret=-1;
                        break;
                    case -1:
                        noTurret=1;
                        break;
                }
            }

            if(noTurret==-1)
            {
                turretOn=false;
            }
            //buton intake
            if (currentGamepad1.a && !previousGamepad1.a) {
                switch(intakeController.currentStatus)
                {
                    case OFF:
                        intakeController.currentStatus = IntakeController.IntakeStatus.COLLECT;
                        break;
                    case COLLECT:
                        intakeController.currentStatus = IntakeController.IntakeStatus.OFF;
                        break;
                    case TRANSFER:
                        intakeController.currentStatus = IntakeController.IntakeStatus.OFF;
                        break;
                }
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                switch(intakeController.currentStatus)
                {
                    case OFF:
                        intakeController.currentStatus = IntakeController.IntakeStatus.REVERSE;
                        break;
                    case REVERSE:
                        intakeController.currentStatus = IntakeController.IntakeStatus.COLLECT;
                        break;

                }
            }

            if(currentGamepad2.dpad_left&&previousGamepad2.dpad_left)
            {
                turretOffset+=1;
            }
            if(currentGamepad2.dpad_right&&previousGamepad2.dpad_right)
            {
                turretOffset-=1;
            }
            //buton mid shoot speed
            if(currentGamepad2.a!=previousGamepad2.a)
            {
                turretOn=true;
                targetVelocity=shoot_mid;
                intakeController.currentStatus= IntakeController.IntakeStatus.COLLECT;
                hoodController.currentStatus= HoodController.HoodStatus.MID;
            }
            //buton far shoot speed
            if(currentGamepad2.x!=previousGamepad2.x)
            {
                turretOn=true;
                targetVelocity=shoot_far;
                intakeController.currentStatus= IntakeController.IntakeStatus.COLLECT;
                hoodController.currentStatus= HoodController.HoodStatus.FAR;

            }
            //buton top of the key shoot speed
            if(currentGamepad2.y!=previousGamepad2.y)
            {
                targetVelocity=shoot_top_key;
                intakeController.currentStatus= IntakeController.IntakeStatus.COLLECT;
                hoodController.currentStatus= HoodController.HoodStatus.TOPKEY;

            }
            //shooting intake and delay
            if(currentGamepad2.b!=previousGamepad2.b&&shootEndTime.seconds()>1&&targetVelocity==shoot_mid)
            {
                rampController.currentStatus= RampController.RampStatus.SHOOT;
                intakeController.currentStatus= IntakeController.IntakeStatus.COLLECT;
                shootTime.reset();
                flywheelOn=true;
            }
            if(currentGamepad2.b!=previousGamepad2.b&&shootEndTime.seconds()>1&&targetVelocity==shoot_far)
            {
                rampController.currentStatus= RampController.RampStatus.SHOOT;

                intakeController.currentStatus= IntakeController.IntakeStatus.TRANSFER;
                shootTime.reset();
                flywheelOn=true;
            }

            if(shootTime.seconds()>delayShoot&&flywheelOn)
            {
                //intakeController.currentStatus= IntakeController.IntakeStatus.COLLECT;
                xDrive=true;
                rampController.currentStatus= RampController.RampStatus.SHOOT;
                shootTransferTime.reset();
                shootTransfer =true;
                flywheelOn=false;
            }

            if(currentGamepad2.b!=previousGamepad2.b&&shootTransfer&&shootTransferTime.seconds()>0.5)
            {
                turretOn=false;
                rampController.currentStatus= RampController.RampStatus.NOSHOOT;
                intakeController.currentStatus = IntakeController.IntakeStatus.OFF;
                targetVelocity=0;
                flywheelOn=false;
                flywheelEnd=true;
                shootEndTime.reset();
                shootTransfer=false;
            }
            if(intakeController.currentStatus== IntakeController.IntakeStatus.COLLECT)
                gamepad1.rumble(0.7,0.7,100);
            if(intakeController.currentStatus== IntakeController.IntakeStatus.OFF)
                gamepad1.stopRumble();




            // Update controllers
            intakeController.update();
            rampController.update();
            hoodController.update();
            turretController.update(turretController.turret.getCurrentPosition(),turret_target_position);




            // === "CommandOpMode thing" (scheduler loop) ===
            // Equivalent to CommandOpMode.super.run() call each loop
            CommandScheduler.getInstance().run();

            // Your original extra update
            robot.pinpoint.update();

            dashTelemetry.addData("intake amps", intakeController.intake.getCurrent(CurrentUnit.AMPS));
            dashTelemetry.addData("shooter1 amps", motor1.getCurrent(CurrentUnit.AMPS));
            dashTelemetry.addData("shooter2 amps", motor.getCurrent(CurrentUnit.AMPS));
            dashTelemetry.addData("turret amps", turretController.turret.getCurrent(CurrentUnit.AMPS));
            dashTelemetry.addData("FR amps", robot.FRmotor.getCurrent(CurrentUnit.AMPS));
            dashTelemetry.addData("BR amps", robot.BRmotor.getCurrent(CurrentUnit.AMPS));
            dashTelemetry.addData("FL amps", robot.FLmotor.getCurrent(CurrentUnit.AMPS));
            dashTelemetry.addData("BL amps", robot.BLmotor.getCurrent(CurrentUnit.AMPS));




            dashTelemetry.addData("Heading", robot.drive.getPose().getHeading());
            dashTelemetry.addData("Robot Pose", robot.drive.getPose());
            dashTelemetry.addData("targetVel", targetVelocity);
            dashTelemetry.addData("curVel",velocity);



            dashTelemetry.addData("degrees to turn", degreesToTurn);
            dashTelemetry.addData("bearingToGoal", bearingToGoal);
            dashTelemetry.addData("turret angle", turretAngle);
            dashTelemetry.addData("robot angle", robot_angle);
            dashTelemetry.addData("angle to turn", angleToTurn);
            dashTelemetry.update();
            telemetry.addData("shoottransfer time",shootTransferTime.seconds());
            telemetry.addData("flywheelOn",flywheelOn);
            telemetry.addData("rampStatus",rampController.currentStatus);

            // Telemetry
            telemetry.addData("Loop Time (ms)", loopTimer.milliseconds());
            loopTimer.reset();

            telemetry.addData("Heading", robot.drive.getPose().getHeading());
            telemetry.addData("Robot Pose", robot.drive.getPose());

            telemetry.addData("Target Chassis Velocity", robot.drive.swerve.getTargetVelocity());
            telemetry.addData("FR Module", robot.drive.swerve.getModules()[0].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[0].getPowerTelemetry());
            telemetry.addData("FL Module", robot.drive.swerve.getModules()[1].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[1].getPowerTelemetry());
            telemetry.addData("BL Module", robot.drive.swerve.getModules()[2].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[2].getPowerTelemetry());
            telemetry.addData("BR Module", robot.drive.swerve.getModules()[3].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[3].getPowerTelemetry());

            telemetry.update();
            robot.updateLoop(telemetryData);

        }



        // Equivalent to end()

        // Optional: cancel commands on stop (nice cleanup)
        CommandScheduler.getInstance().reset();


    }

    public double turnTurretByDegrees(double degrees) {
        // Get current position and calculate target
        double currentTicks = turretController.turret.getCurrentPosition();
        double targetTicks = currentTicks + degreesToEncoderTicks(degrees)+turretOffset;


        // Check soft limits
        double targetAngle = getTurretAngleDegrees() + degrees;

        dashTelemetry.addData("currentTicks", currentTicks);
        dashTelemetry.addData("targetTicks", targetTicks);
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

