package org.firstinspires.ftc.teamcode.globals;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class HoodController {
    public enum HoodStatus{
        INIT,
        FAR,
        MID,
        CLOSE,
        TOPKEY;
    }
    public HoodStatus currentStatus= HoodStatus.INIT;
    public HoodStatus previousStatus=null;

    public static double hoodInitPosition=1;
    public static double hoodFarPosition=0.5;
    public static double hoodMidPosition=0.63;
    public static double hoodTopKeyPosition=0.55;
    public static double hoodClosePosition=0.75;



    public Servo hood = null;

    public HoodController(RobotMap robot){this.hood=robot.hood;}
    public void update()
    {
        if(currentStatus!=previousStatus){
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INIT:
                {
                    this.hood.setPosition(hoodInitPosition);
                    break;
                }
                case FAR:
                {
                    this.hood.setPosition(hoodFarPosition);
                    break;
                }
                case MID:
                {
                    this.hood.setPosition(hoodMidPosition);
                    break;
                }
                case TOPKEY:
                {
                    this.hood.setPosition(hoodTopKeyPosition);
                    break;
                }
                case CLOSE:
                {
                    this.hood.setPosition(hoodClosePosition);
                    break;
                }
            }
        }
    }
}

