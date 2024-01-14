package org.firstinspires.ftc.teamcode.based.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.RobotHardwareConfig;

@Config
public class LiftSubsystem extends SubsystemBase {
    public static double
        kP = 0,
        kI = 0,
        kD = 0;

    public static double kF = 0;

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;

    private PIDController controller;

    private int target, currentPosition;

    private double lastPower;

    private boolean resetting = false;

    public LiftSubsystem(HardwareMap hardwareMap){
        super();

        this.leftMotor = hardwareMap.get(DcMotorEx.class, RobotHardwareConfig.Lift.LEFT_MOTOR_STRING);
        this.rightMotor = hardwareMap.get(DcMotorEx.class, RobotHardwareConfig.Lift.RIGHT_MOTOR_STRING);

        this.controller = new PIDController(kP, kI, kD);
    }

    public int getCurrentPosition(){
        return this.leftMotor.getCurrentPosition();
    }

    public int getTargetPosition(){
        return this.target;
    }

    public void doResetMovement(){
        this.resetting = true;
        this.setPower(Constants.Lift.RESET_SPEED);
    }

    public void reset(){
        this.setPower(0);
        this.target = Constants.Lift.GROUND_POSITION;
        controller.reset();
        this.resetting = false;
    }

    public void setTarget(int target){
        if (this.target != target) this.target = target;
    }

    private void setPower(double power){
        if (Double.compare(power, this.lastPower) != 0){
            this.leftMotor.setPower(power);
            this.rightMotor.setPower(power);
            this.lastPower = power;
        }
    }

    @Override
    public void periodic(){
        this.currentPosition = this.leftMotor.getCurrentPosition();

        if (!resetting){
            double power = controller.calculate(this.currentPosition, this.target);
            this.setPower(power + kF);
        }
    }

}
