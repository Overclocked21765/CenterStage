package org.firstinspires.ftc.teamcode.based.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.RobotHardwareConfig;

public class ArmSubsystem extends SubsystemBase {
    public enum ArmStates{
        INTAKE,
        DEPOSIT,
        DEPOSIT_ALT,
        NAN
    }

    private ArmStates state;

    private Servo
        leftServo,
        rightServo;

    public ArmSubsystem(HardwareMap hardwareMap){
        this.leftServo = hardwareMap.get(Servo.class, RobotHardwareConfig.Arm.LEFT_SERVO);
        this.rightServo = hardwareMap.get(Servo.class, RobotHardwareConfig.Arm.RIGHT_SERVO);

        this.leftServo.setDirection(RobotHardwareConfig.Arm.ARM_LEFT_DIRECTION);
        this.rightServo.setDirection(RobotHardwareConfig.Arm.ARM_RIGHT_DIRECTION);

        state = ArmStates.INTAKE;
    }

    public void setIntakePosition(){
        this.leftServo.setPosition(Constants.Arm.INTAKE_POSITION_LEFT);
        this.rightServo.setPosition(Constants.Arm.INTAKE_POSITION_RIGHT);
        state = ArmStates.INTAKE;
    }

    public void setDepositPosition(){
        this.leftServo.setPosition(Constants.Arm.OUTTAKE_POSITION_LEFT);
        this.rightServo.setPosition(Constants.Arm.OUTTAKE_POSITION_RIGHT);
        state = ArmStates.DEPOSIT;
    }

    public void setAltDepositPosition(){
        this.leftServo.setPosition(Constants.Arm.OUTTAKE_ALT_POSITION_LEFT);
        this.rightServo.setPosition(Constants.Arm.OUTTAKE_ALT_POSITION_RIGHT);
        state = ArmStates.DEPOSIT_ALT;
    }

    public void setPositions(double l, double r){
        this.leftServo.setPosition(l);
        this.rightServo.setPosition(r);
        state = ArmStates.NAN;
    }


    public ArmStates getState() {
        return this.state;
    }

    public void setState(ArmStates state){
        this.state = state;
    }
}
