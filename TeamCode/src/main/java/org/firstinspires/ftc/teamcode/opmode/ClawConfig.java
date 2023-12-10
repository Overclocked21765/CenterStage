package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.ejml.equation.Operation;
import org.firstinspires.ftc.teamcode.common.RobotHardwareConfig;

@Config
@TeleOp(group = "Hardware")
public class ClawConfig extends OpMode {
    public static double pos = 0;
    public static int reversable;

    Servo servo;

    @Override
    public void init(){
        servo = hardwareMap.get(Servo.class, RobotHardwareConfig.Claw.CLAW_STRING);

        reversable = 0;

    }

    @Override
    public void loop(){
        servo.setDirection((reversable == 0) ? Servo.Direction.FORWARD : Servo.Direction.REVERSE);
        servo.setPosition(pos);
    }
}
