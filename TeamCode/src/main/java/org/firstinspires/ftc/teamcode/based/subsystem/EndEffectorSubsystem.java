package org.firstinspires.ftc.teamcode.based.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.RobotHardwareConfig;

public class EndEffectorSubsystem extends SubsystemBase {
    private Servo
        leftServo,
        rightServo;

    private Servo wristServo;

    public EndEffectorSubsystem(HardwareMap hardwareMap){
        super();
        this.leftServo = hardwareMap.get(Servo.class, RobotHardwareConfig.EndEffector.CLAW_LEFT_STRING);
        this.rightServo = hardwareMap.get(Servo.class, RobotHardwareConfig.EndEffector.CLAW_RIGHT_STRING);

        this.leftServo.setDirection(RobotHardwareConfig.EndEffector.CLAW_LEFT_DIRECTION);
        this.rightServo.setDirection(RobotHardwareConfig.EndEffector.CLAW_RIGHT_DIRECTION);
    }

    public void grabLeft(){
        this.leftServo.setPosition(Constants.EndEffector.LEFT_CLOSE);
    }

    public void grabRight(){
        this.rightServo.setPosition(Constants.EndEffector.RIGHT_CLOSE);
    }

    public void grab(){
        this.grabLeft();
        this.grabRight();
    }

    public void releaseLeft(){
        this.leftServo.setPosition(Constants.EndEffector.LEFT_OPEN);
    }

    public void releaseRight(){
        this.rightServo.setPosition(Constants.EndEffector.RIGHT_OPEN);
    }

    public void release(){
        this.releaseLeft();
        this.releaseRight();
    }

    public void moveToIntake(){
        wristServo.setPosition(Constants.EndEffector.WRIST_INTAKE);
    }

    public void stow(){
        wristServo.setPosition(Constants.EndEffector.WRIST_STOW);
    }

    public void moveToAltDeposit(){
        wristServo.setPosition(Constants.EndEffector.WRIST_ALT_DEPOSIT);
    }
}
