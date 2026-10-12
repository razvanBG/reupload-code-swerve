package org.firstinspires.ftc.teamcode.globals;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class RampController {
    public enum RampStatus{
        SHOOT,
        NOSHOOT;
    }
    public RampStatus currentStatus= RampStatus.NOSHOOT;
    public RampStatus previousStatus=null;

    public static double rampOffPosition=0.85;
    public static double rampOnPosition=0.78;

    public Servo ramp = null;

    public RampController(RobotMap robot){this.ramp=robot.ramp;}
    public void update()
    {
        if(currentStatus!=previousStatus){
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case NOSHOOT:
                {
                    this.ramp.setPosition(rampOffPosition);
                    break;
                }
                case SHOOT:
                {
                    this.ramp.setPosition(rampOnPosition);
                    break;
                }
            }
        }
    }
}

