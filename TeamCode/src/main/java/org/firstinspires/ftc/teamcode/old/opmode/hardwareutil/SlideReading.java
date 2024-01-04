package org.firstinspires.ftc.teamcode.old.opmode.hardwareutil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.old.RobotHardwareConfig;

@Config
@TeleOp(group = "Hardware")
public class SlideReading extends OpMode {
    public static int target = 0;
    DcMotorEx motor;

    @Override
    public void init(){
        motor = hardwareMap.get(DcMotorEx.class, RobotHardwareConfig.Lift.LIFT_STRING);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop(){
        motor.setTargetPosition(target);
        telemetry.addData("pos: ", motor.getCurrentPosition());
        telemetry.addData("target: ", target);
    }
}
