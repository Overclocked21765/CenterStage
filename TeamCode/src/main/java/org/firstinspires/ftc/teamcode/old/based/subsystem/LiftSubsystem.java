package org.firstinspires.ftc.teamcode.old.based.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.old.RobotHardwareConfig;

@Deprecated
public class LiftSubsystem extends SubsystemBase {
    private DcMotorEx liftMotor;

    private int targetPosition, currentPosition, lastTargetPosition;

    public static int MAX = 3650;
    public static int MIN = 60;

    public static int TICKS_PER_REQUEST = 225;

    public boolean resetting;



    public LiftSubsystem(HardwareMap hardwareMap){
        super();

        targetPosition = 0;
        lastTargetPosition = 0;
        resetting = false;

        this.liftMotor = hardwareMap.get(DcMotorEx.class, RobotHardwareConfig.Lift.LIFT_STRING);
        this.liftMotor.setZeroPowerBehavior(RobotHardwareConfig.Lift.DEFAULT_BEHAVIOR);
        this.liftMotor.setDirection(RobotHardwareConfig.Lift.LIFT_DIRECTION);
        this.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotor.setTargetPosition(targetPosition);
        this.liftMotor.setPower(0.99);
        this.liftMotor.setMode(RobotHardwareConfig.Lift.DEFAULT_RUNMODE);

    }

    public void move(double dU){
        int nextPosition = (int) (Math.round(dU * TICKS_PER_REQUEST) + this.currentPosition);
        if (dU > 0 && nextPosition < MAX){
            this.setTargetPosition(nextPosition);
        } else if (dU < 0 && nextPosition > MIN){
            this.setTargetPosition(nextPosition);
        }

        ;
    }

    public void setTargetPosition(int target){
        if (this.targetPosition != target) this.targetPosition = target;
        this.liftMotor.setTargetPosition(this.targetPosition);
    }

    public void resetMovement(){
        this.setTargetPosition(this.targetPosition - 100);
    }

    public void reset(){
        this.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setTargetPosition(0);
        this.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getTargetPosition(){
        return this.targetPosition;
    }

    public int getCurrentPosition() {
        return currentPosition;
    }

    @Override
    public void periodic(){
        this.currentPosition = this.liftMotor.getCurrentPosition();

        if (!resetting){
            if (this.currentPosition > MAX) this.setTargetPosition(MAX);
            else if (this.currentPosition < MIN) this.setTargetPosition(MIN);
        }

    }
}
