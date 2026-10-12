package org.firstinspires.ftc.teamcode.globals;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
public class TurretController {
    public enum TurretStatus{
        INIT,
        RUNTO;
    }
    public TurretStatus currentStatus = TurretStatus.INIT;
    public TurretStatus previousStatus = null;

    public DcMotorEx turret = null;

    public static int init_position=0;

    SimplePIDController turretPID = null;
    public static double Kp = 0.015;
    public static double Ki = 0;
    public static double Kd = 0.1;
    public static double PowerCap = 1;
    public static double maxSpeed = 1;

    public TurretController(RobotMap robot) {
        this.turret = robot.turret;
        turretPID = new SimplePIDController(Kp, Ki, Kd);
        turretPID.targetValue = init_position;
        turretPID.maxOutput= maxSpeed;
    }

    public void update(double turretPosition,double runto_target){
        double powerTurret = turretPID.update(turretPosition);
        powerTurret = Math.max(-PowerCap,Math.min(powerTurret,PowerCap));
        this.turret.setPower(powerTurret);
        if(currentStatus!=previousStatus || currentStatus== TurretStatus.RUNTO)
        {
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INIT:
                {
                    turretPID.targetValue=init_position;
                    break;
                }

                case RUNTO:
                {
                    turretPID.targetValue=runto_target;
                    break;
                }
            }
        }
        if (Kp!=turretPID.p || Kd!=turretPID.d || Ki!=turretPID.i || maxSpeed !=turretPID.maxOutput)
        {
            turretPID.p = Kp;
            turretPID.d = Kd;
            turretPID.i = Ki;
            turretPID.maxOutput = maxSpeed;
        }
    }
}
