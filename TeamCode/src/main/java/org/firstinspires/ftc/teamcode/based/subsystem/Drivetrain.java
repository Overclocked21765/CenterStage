package org.firstinspires.ftc.teamcode.based.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.Algorithms;

@Config
public class Drivetrain extends SubsystemBase {
    private enum State{
        PID,
        ROTATE,
        SETTING
    }

    private DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private IMU imu;
    private Telemetry telemetry;

    public static double kP = 0.015;
    public static double kI = 0;
    public static double kD = 0;
    public static double margin = 2;
    public static double maxAllowedError = 350;
    protected double targetHeading;
    protected boolean firstTimeAfterStickRelease;
    PIDController controller;

    double maxError;

    public enum States{
        PID,
        ROTATING,
        SETTING
    }

    States state;



    public void update(double leftX, double leftY, double rightX){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double heading = orientation.getYaw(AngleUnit.DEGREES);
        double error = heading - targetHeading;

        double headingUnused = heading;
        if (error > maxAllowedError){
            error -= 360;
            headingUnused -= 360;
        } else if (error < -maxAllowedError){
            error += 360;
            headingUnused += 360;
        }

        if (Math.abs(error) > maxError){
            maxError = Math.abs(error);
        }

        if (rightX == 0) {
            if (firstTimeAfterStickRelease){
                targetHeading = heading;
                error = heading - targetHeading;
                firstTimeAfterStickRelease = false;
            }
            if (Math.abs(error) > margin) {
                drive(controller.calculate(heading, targetHeading), leftX, leftY, heading);
            } else {
                drive(0, leftX, leftY, heading);
            }
            telemetry.addData("Mode: ", "PID");
        } else {
            drive(rightX, leftX, leftY, heading);
            firstTimeAfterStickRelease = true;
            telemetry.addData("Mode: ", "Rotating");
        }


        telemetry.addData("Heading: ", heading);
        telemetry.addData("Heading unused: ", headingUnused);
        telemetry.addData("Error: ", error);
        telemetry.addData("Target: ", targetHeading);
        telemetry.addData("Max Error: ", maxError);

    }

    public void resetYaw(){
        imu.resetYaw();
        targetHeading = 0;
    }



    public Drivetrain(HardwareMap hwMap, DcMotor.ZeroPowerBehavior zeroPowerBehavior, Telemetry telemetry){
        targetHeading = 0;
        state = States.PID;
        firstTimeAfterStickRelease = false;
        controller = new PIDController(kP, kI, kD);
        maxError = 0;
        this.telemetry = telemetry;
        //This sets up the motors it also tells us what the names of our motors are in a string format
        frontLeftMotor = hwMap.get(DcMotorEx.class, "Front_Left");
        frontRightMotor = hwMap.get(DcMotorEx.class, "Front_Right");
        backLeftMotor = hwMap.get(DcMotorEx.class, "Back_Left");
        backRightMotor = hwMap.get(DcMotorEx.class, "Back_Right");
        //This makes our motors run using encoders
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //This sets the motors default direction that it spins
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //This makes sure that when the motors are given no input they will not do anything
        frontLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        frontRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
        backLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        backRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
        //This is gyro/imu stuff
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        this.resetYaw();
    }



    public void drive(double rotation, double strafe, double forwardPower, double heading){
        double [] motorArraySpeeds = Algorithms.returnMecanumValues(rotation, strafe, forwardPower, heading, 0.8);

        frontLeftMotor.setPower(motorArraySpeeds[0]);
        frontRightMotor.setPower(motorArraySpeeds[1]);
        backLeftMotor.setPower(motorArraySpeeds[2]);
        backRightMotor.setPower(motorArraySpeeds[3]);
    }


    public void drive(double leftX, double leftY, double rightX){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double heading = orientation.getYaw(AngleUnit.DEGREES);
        double error = heading - targetHeading;
        if (error > maxAllowedError){
            error -= 360;
            heading -= 360;
        } else if (error < -maxAllowedError){
            error += 360;
            heading += 360;
        }

        if (Math.abs(error) > maxError){
            maxError = Math.abs(error);
        }

        if (rightX == 0) {
            if (firstTimeAfterStickRelease){
                targetHeading = heading;
                error = heading - targetHeading;
                firstTimeAfterStickRelease = false;
            }
            if (Math.abs(error) > margin) {
                drive(controller.calculate(heading, targetHeading), leftX, leftY, heading);
            } else {
                drive(0, leftX, leftY, heading);
            }
            telemetry.addData("Mode: ", "PID");
        } else {
            drive(rightX, leftX, leftY, heading);
            firstTimeAfterStickRelease = true;
            telemetry.addData("Mode: ", "Rotating");
        }


        telemetry.addData("Heading: ", heading);
        telemetry.addData("Error: ", error);
        telemetry.addData("Target: ", targetHeading);
        telemetry.addData("Max Error: ", maxError);

    }




    public void setTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
    }
}