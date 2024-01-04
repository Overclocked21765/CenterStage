package org.firstinspires.ftc.teamcode.old.example;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PIDDemonstration extends OpMode {

    public static double kP = 0, kI = 0, kD = 0;
    public static double kF = 0;
    public static double target = 0;

    public static double ticksPerDegrees = 384.5 / 360;


    DcMotorEx motor;

    PIDController controller;

    @Override
    public void init(){
        controller = new PIDController(kP, kI, kD);
        motor = hardwareMap.get(DcMotorEx.class, "Motor");
    }

    @Override
    public void loop(){
        controller.setPID(kP, kI, kD);
        double position = motor.getCurrentPosition();
        double ff = kF * Math.cos(Math.toRadians(position * ticksPerDegrees));
        double power = controller.calculate(position, target);

        motor.setPower(power + ff);
    }
}
