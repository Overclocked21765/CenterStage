package org.firstinspires.ftc.teamcode.based.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.RobotHardwareConfig;

@Config
public class ClawSubsystem {
    private Servo clawSwervo;

    public static double OPEN = 0.65;
    public static double CLOSE = 0.4;

    public ClawSubsystem(HardwareMap hardwareMap){
        super();

        clawSwervo = hardwareMap.get(Servo.class, RobotHardwareConfig.Claw.CLAW_STRING);
        clawSwervo.setDirection(RobotHardwareConfig.Claw.CLAW_DIRECTION);
    }

    public void grab(){
        this.clawSwervo.setPosition(CLOSE);
    }

    public void release(){
        this.clawSwervo.setPosition(OPEN);
    }
}
