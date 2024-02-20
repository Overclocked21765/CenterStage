package org.firstinspires.ftc.teamcode.based.subsystem;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.RobotHardwareConfig;

public class EndEffectorSubsystem extends SubsystemBase {
    public enum EffectorStates{
        OPEN,
        CLOSED,
    }

    private Servo
        leftServo,
        rightServo;

    private Servo wristServo;

    private EffectorStates leftState = EffectorStates.OPEN;
    private EffectorStates rightState = EffectorStates.OPEN;

    private Telemetry telemetry;

    private int called = 0;

    public EndEffectorSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        super();
        this.leftServo = hardwareMap.get(Servo.class, RobotHardwareConfig.EndEffector.CLAW_LEFT_STRING);
        this.rightServo = hardwareMap.get(Servo.class, RobotHardwareConfig.EndEffector.CLAW_RIGHT_STRING);
        this.wristServo = hardwareMap.get(Servo.class, RobotHardwareConfig.EndEffector.WRIST_SERVO);

        this.leftServo.setDirection(RobotHardwareConfig.EndEffector.CLAW_LEFT_DIRECTION);
        this.rightServo.setDirection(RobotHardwareConfig.EndEffector.CLAW_RIGHT_DIRECTION);
        this.wristServo.setDirection(Servo.Direction.FORWARD);

        this.telemetry = telemetry;
    }

    public void grabLeft(){
        this.leftServo.setPosition(Constants.EndEffector.LEFT_CLOSE);
        this.leftState = EffectorStates.CLOSED;
    }

    public void grabRight(){
        this.rightServo.setPosition(Constants.EndEffector.RIGHT_CLOSE);
        this.rightState = EffectorStates.CLOSED;
    }

    public void grab(){
        this.grabLeft();
        this.grabRight();
    }

    public void releaseLeft(){
        this.leftServo.setPosition(Constants.EndEffector.LEFT_OPEN);
        this.leftState = EffectorStates.OPEN;
    }

    public void releaseRight(){
        this.rightServo.setPosition(Constants.EndEffector.RIGHT_OPEN);
        this.rightState = EffectorStates.OPEN;
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

    public void passThrough(){
        wristServo.setPosition(Constants.EndEffector.WRIST_PASSTRHOUGH);
    }

    public void moveToAltDeposit(){
        wristServo.setPosition(Constants.EndEffector.WRIST_ALT_DEPOSIT);
    }

    public void moveToDeposit(){
        wristServo.setPosition(Constants.EndEffector.WRIST_DEPOSIT);
    }

    public void updateClaw(){

        if (leftState == EffectorStates.OPEN || rightState == EffectorStates.OPEN) {
            this.grab();
        } else {
            this.release();
        }

    }

    public void updateClawLeft(){
        if (leftState == EffectorStates.OPEN) this.grabLeft();
        else this.releaseLeft();
    }

    public void updateClawRight(){
        if (rightState == EffectorStates.OPEN) this.grabRight();
        else this.releaseRight();
    }

    @Override
    public void periodic(){
        telemetry.addLine("\n---End Effector-----------------------");
        telemetry.addData("left state: ", this.leftState);
        telemetry.addData("right state: ", this.rightState);

        telemetry.addData("called: ", called);
    }
}
