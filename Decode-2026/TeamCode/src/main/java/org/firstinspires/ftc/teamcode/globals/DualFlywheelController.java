package org.firstinspires.ftc.teamcode.globals;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class DualFlywheelController {

    public enum ShooterState { IDLE, SPIN_UP, READY, SHOOT }

    public final DcMotorEx motorR;
    public final DcMotorEx motorL;

    public ShooterState currentState = ShooterState.IDLE;

    // PID constants
    public static double kP = 0.003;
    public static double kI = 0;
    public static double kD = 0.004;
    public static double kF = 0.000452;

    public static double integral = 0;
    public static double lastError = 0;

    public static double targetVelocity = 0; // ticks/sec

    // Stability check
    public static int readyCounter = 0;
    public static final int READY_THRESHOLD_COUNT = 5; // must be stable 5 cycles
    public static final double VELOCITY_TOLERANCE = 20; // ticks/sec

    public DualFlywheelController(HardwareMap hwMap, String rightMotorName, String leftMotorName) {
        motorR = hwMap.get(DcMotorEx.class, rightMotorName);
        motorL = hwMap.get(DcMotorEx.class, leftMotorName);

        motorR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorR.setDirection(DcMotorEx.Direction.FORWARD);
        motorL.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void setTargetVelocity(double rpm) {
        targetVelocity = rpm * 28 / 60.0; // convert RPM to ticks/sec
        currentState = ShooterState.SPIN_UP;
        readyCounter = 0;
    }

    public void stop() {
        targetVelocity = 0;
        motorR.setPower(0);
        motorL.setPower(0);
        currentState = ShooterState.IDLE;
        readyCounter = 0;
    }

    public void shoot() {
        if (currentState == ShooterState.READY) {
            currentState = ShooterState.SHOOT;
            // TODO: add servo firing mechanism here
        }
    }

    public void update() {
        double currentVelocity = getRawVelocity(); // avg ticks/sec
        double error = targetVelocity - currentVelocity;
        integral += error;
        double derivative = error - lastError;
        lastError = error;

        double power = kP * error + kI * integral + kD * derivative + kF * targetVelocity;
        power = Math.max(0, Math.min(1.0, power));

        motorR.setPower(power);
        motorL.setPower(power);

        // State machine
        switch(currentState) {
            case SPIN_UP:
                if (Math.abs(currentVelocity - targetVelocity) < VELOCITY_TOLERANCE) {
                    readyCounter++;
                    if (readyCounter >= READY_THRESHOLD_COUNT) {
                        currentState = ShooterState.READY;
                        readyCounter = 0;
                    }
                } else {
                    readyCounter = 0; // reset if it drifts
                }
                break;

            case SHOOT:
                currentState = ShooterState.SPIN_UP;
                break;

            case IDLE:
            case READY:
                break;
        }
    }

    // Private: raw velocity (ticks/sec)
    private double getRawVelocity() {
        return (motorR.getVelocity() + motorL.getVelocity()) / 2.0;
    }

    // Public: return RPM
    public double getCurrentVelocity() {
        return getRawVelocity() * 60 / 28.0;
    }

    public double getTargetVelocity() {
        return targetVelocity * 60 / 28.0;
    }

    public ShooterState getCurrentState() { return currentState; }


}
