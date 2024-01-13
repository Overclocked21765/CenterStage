package org.firstinspires.ftc.teamcode.based.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.RobotHardwareConfig;

public class ArmSubsystem extends SubsystemBase {
    private Servo
        leftServo,
        rightServo;

    public ArmSubsystem(HardwareMap hardwareMap){
        this.leftServo = hardwareMap.get(Servo.class, RobotHardwareConfig.Arm.LEFT_SERVO);
        this.rightServo = hardwareMap.get(Servo.class, RobotHardwareConfig.Arm.RIGHT_SERVO);

        this.leftServo.setDirection(RobotHardwareConfig.Arm.ARM_LEFT_DIRECTION);
        this.rightServo.setDirection(RobotHardwareConfig.Arm.ARM_RIGHT_DIRECTION);
    }

    public void setIntakePosition(){

    }

    public void setDepositPosition(){

    }

    public void setAltDepositPosition(){

    }
}
