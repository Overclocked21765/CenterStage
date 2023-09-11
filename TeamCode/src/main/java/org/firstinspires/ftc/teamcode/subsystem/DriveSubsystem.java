package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.common.Algorithms.returnMecanumValues;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.Constants;

import java.util.function.DoubleSupplier;


public class DriveSubsystem extends SubsystemBase {
    private DcMotorEx fL, fR, bL, bR;
    private MotorWrapper fl, fr, bl, br;
    private IMU imu;
    private DoubleSupplier leftX, leftY, rightX;

    public DriveSubsystem(HardwareMap hwMap, GamepadEx gamepad){
        fL = hwMap.get(DcMotorEx.class, "Front_left_motor");
        fR = hwMap.get(DcMotorEx.class, "Front_right_motor");
        bL = hwMap.get(DcMotorEx.class, "Back_left_motor");
        bR = hwMap.get(DcMotorEx.class, "Back_right_motor");

        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl = new MotorWrapper(fL);
        fr = new MotorWrapper(fR);
        bl = new MotorWrapper(bL);
        br = new MotorWrapper(bR);



        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        leftX = gamepad::getLeftX;
        leftY = gamepad::getLeftY;
        rightX = gamepad::getRightX;
    }

    public void setPowers(double[] powers){
        fl.setPower(powers[0]);
        fr.setPower(powers[1]);
        bl.setPower(powers[2]);
        br.setPower(powers[3]);
    }

    public void drive(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double[] powers = returnMecanumValues(leftY.getAsDouble(), leftX.getAsDouble(), rightX.getAsDouble(), orientation.getYaw(AngleUnit.DEGREES), Constants.Drive.DRIVE_POWER_MODIFIER);
    }

    public class MotorWrapper{
        public DcMotorEx motor;
        double lastPower;

        public MotorWrapper(DcMotorEx motor){
            this.motor = motor;
            lastPower = 0;
        }

        public void setPower(double power){
            if (power != lastPower){
                motor.setPower(power);
                lastPower = power;
            }
        }
    }

}
