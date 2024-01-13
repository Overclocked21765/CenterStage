package org.firstinspires.ftc.teamcode.opmode.hardwareutil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(group = "Hardware")
public class HardwareServo extends OpMode {
    private Servo servo;
    public static double pos = 0;
    public static int reverse = 0;
    public static String str = "Claw Left";

    @Override
    public void init() {
        this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);
    }

    @Override
    public void init_loop(){
        telemetry.addData("Str: ", str);
    }

    @Override
    public void start(){
        this.servo = this.hardwareMap.get(Servo.class, str);
    }

    @Override
    public void loop() {
        if (reverse == 0) this.servo.setDirection(Servo.Direction.FORWARD);
        else if (reverse == 1) this.servo.setDirection(Servo.Direction.REVERSE);

        this.servo.setPosition(pos);

        telemetry.addData("Dir: ", this.servo.getDirection());
    }
}
