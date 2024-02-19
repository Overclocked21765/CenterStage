package org.firstinspires.ftc.teamcode.old.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.based.subsystem.HeadingPID;

@TeleOp
@Disabled
public class HeadingPIDOp extends OpMode {
    HeadingPID drive;

    boolean buttonAlreadyPressed;

    @Override
    public void init(){
        drive = new HeadingPID(0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive.init(hardwareMap, DcMotor.ZeroPowerBehavior.FLOAT, telemetry);
        buttonAlreadyPressed = false;
    }

    @Override
    public void loop(){
        drive.updateExperimental(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
//double leftX, double leftY, double rightX
        if (gamepad1.y && !buttonAlreadyPressed){
            drive.resetYaw();
        }
        buttonAlreadyPressed = gamepad1.y;
    }
}
