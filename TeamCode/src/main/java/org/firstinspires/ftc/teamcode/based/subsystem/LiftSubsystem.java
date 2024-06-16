package org.firstinspires.ftc.teamcode.based.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.RobotHardwareConfig;

@Config
public class LiftSubsystem extends SubsystemBase {
    public static double
        kP = 0.025,
        kI = 0,
        kD = 0.0004;

    public static double kF = 0;

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;

    private PIDController controller;

    private int target, currentPosition;

    private double lastPower;

    private boolean resetting = false;
    private boolean triggerMoving = false;

    private Telemetry telemetry;

    public LiftSubsystem(HardwareMap hardwareMap){
        super();

        this.leftMotor = hardwareMap.get(DcMotorEx.class, RobotHardwareConfig.Lift.LEFT_MOTOR_STRING);
        this.rightMotor = hardwareMap.get(DcMotorEx.class, RobotHardwareConfig.Lift.RIGHT_MOTOR_STRING);

        this.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.leftMotor.setDirection(RobotHardwareConfig.Lift.LEFT_DIRECTION);
        this.rightMotor.setDirection(RobotHardwareConfig.Lift.RIGHT_DIRECTION);

        this.controller = new PIDController(kP, kI, kD);
    }

    public LiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        this(hardwareMap);
        this.telemetry = telemetry;
    }

    public int getCurrentPosition(){
        return this.leftMotor.getCurrentPosition();
    }

    public int getTargetPosition(){
        return this.target;
    }

    public void doResetMovement(){
        this.resetting = true;
        this.setPower(Constants.ConstantsLift.RESET_SPEED);
    }

    public void reset(){
        this.setPower(0);
        this.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.target = Constants.ConstantsLift.GROUND_POSITION;
        controller.reset();
        this.resetting = false;
    }

    public void setTarget(int target){
        this.target = target;
    }

    public void move(double du){
        if (Double.compare(0, du) != 0) {
            int add = (int) (Math.round(du * Constants.ConstantsLift.TICK_PER_REQ));
            if (this.target + add > Constants.ConstantsLift.MAX) this.target = Constants.ConstantsLift.MAX;
            else if (this.target + add < Constants.ConstantsLift.GROUND_POSITION)
                this.target = Constants.ConstantsLift.GROUND_POSITION;
            else this.target += add;
        }
    }

    public void moveManually(double speed){
        if (this.currentPosition < Constants.ConstantsLift.MAX && this.currentPosition > Constants.ConstantsLift.GROUND_POSITION){
            triggerMoving = true;
            this.setPower(speed);
        } else {
            triggerMoving = false;
        }
    }

    public void stopMovement(){
        triggerMoving = false;
        this.target = currentPosition;
    }


    private void setPower(double power){
        this.leftMotor.setPower(power);
        this.rightMotor.setPower(power);
        this.telemetry.addData("lift power: ", power);
    }

    @Override
    public void periodic(){
        telemetry.addLine("\n--Lift---------------------------------");
        this.currentPosition = this.leftMotor.getCurrentPosition();

        this.controller.setPID(kP, kI, kD);

        if (!resetting && !triggerMoving){
            double power = controller.calculate(this.currentPosition, this.target);
            this.setPower(power + kF);
        }

        telemetry.addData("current position: ", this.currentPosition);

        telemetry.addData("triggerMoving?: ", triggerMoving);
        this.telemetry.addData("target: ", this.target);
    }

}
