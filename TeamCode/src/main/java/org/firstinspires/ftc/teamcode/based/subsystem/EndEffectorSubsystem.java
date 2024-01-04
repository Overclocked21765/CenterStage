package org.firstinspires.ftc.teamcode.based.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.RobotHardwareConfig;

public class EndEffectorSubsystem extends SubsystemBase {
    private Servo
        leftServo,
        rightServo;

    private Servo wristServo;

    public EndEffectorSubsystem(HardwareMap hardwareMap){
        this.leftServo = hardwareMap.get(Servo.class, RobotHardwareConfig.Claw.CLAW_LEFT_STRING);
        this.rightServo = hardwareMap.get(Servo.class, RobotHardwareConfig.Claw.CLAW_RIGHT_STRING);

        this.leftServo.setDirection(RobotHardwareConfig.Claw.CLAW_LEFT_DIRECTION);
        this.rightServo.setDirection(RobotHardwareConfig.Claw.CLAW_RIGHT_DIRECTION);
    }

    public void grab(){

    }

    public void releaseLeft(){

    }

    public void releaseRight(){

    }

    public void release(){

    }
}
