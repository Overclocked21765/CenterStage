package org.firstinspires.ftc.teamcode.old.based.subsystem;

import static org.firstinspires.ftc.teamcode.common.util.Algorithms.returnMecanumValues;

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
import org.firstinspires.ftc.teamcode.common.Constants;

@Deprecated
public class Drivetrain extends SubsystemBase{
    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;
    //This is gyro stuff or imu stuff
    private IMU imu;
    private Telemetry telemetry;

    public static double kP = 0.015;
    public static double kI = 0;
    public static double kD = 0;
    public static double margin = 1.5;
    public static double maxAllowedError = 350;
    private double targetHeading;
    private boolean firstTimeAfterStickRelease;
    private PIDController controller;

    double maxError;

    public enum States{
        PID,
        ROTATING,
        SETTING
    }

    HeadingPID.States state;

    public void init(HardwareMap hwMap, DcMotor.ZeroPowerBehavior zeroPowerBehavior, Telemetry telemetry){
        this.telemetry = telemetry;
        //This sets up the motors it also tells us what the names of our motors are in a string format
        frontLeftMotor = hwMap.get(DcMotorEx.class, "Front_Left");
        frontRightMotor = hwMap.get(DcMotorEx.class, "Front_Right");
        backLeftMotor = hwMap.get(DcMotorEx.class, "Back_Left");
        backRightMotor = hwMap.get(DcMotorEx.class, "Back_Right");
        //This makes our motors run using encoders
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //This sets the motors default direction that it spins
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
    }

    public void drive(double rotation, double strafe, double forwardPower, double heading, double scalar){
        double [] motorArraySpeeds = returnMecanumValues(rotation, strafe, forwardPower, heading, scalar);

        frontLeftMotor.setPower(motorArraySpeeds[0]);
        frontRightMotor.setPower(motorArraySpeeds[1]);
        backLeftMotor.setPower(motorArraySpeeds[2]);
        backRightMotor.setPower(motorArraySpeeds[3]);
    }

    public Drivetrain(double heading, HardwareMap hardwareMap, Telemetry telemetry){
        super();
        targetHeading = heading;
        state = HeadingPID.States.PID;
        firstTimeAfterStickRelease = false;
        controller = new PIDController(kP, kI, kD);
        maxError = 0;
        this.init(hardwareMap, DcMotor.ZeroPowerBehavior.FLOAT, telemetry);
        this.resetYaw();

    }

    public void update(double leftX, double leftY, double rightX){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double heading = orientation.getYaw(AngleUnit.DEGREES);
        double error = heading - targetHeading;
        if (error > maxAllowedError){
            error -= 360;
        } else if (error < -maxAllowedError){
            error += 360;
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
                drive(controller.calculate(heading, targetHeading), leftX, leftY, heading, Constants.Drive.DRIVE_POWER_MODIFIER);
            } else {
                drive(0, leftX, leftY, heading, Constants.Drive.DRIVE_POWER_MODIFIER);
            }
            telemetry.addData("Mode: ", "PID");
        } else {
            drive(rightX, leftX, leftY, heading, Constants.Drive.DRIVE_POWER_MODIFIER);
            firstTimeAfterStickRelease = true;
            telemetry.addData("Mode: ", "Rotating");
        }


        telemetry.addData("Heading: ", heading);
        telemetry.addData("Error: ", error);
        telemetry.addData("Target: ", targetHeading);
        telemetry.addData("Max Error: ", maxError);

    }

    public void updateExperimental(double leftX, double leftY, double rightX){
        telemetry.addLine("Experimental drive enabled");
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double heading = orientation.getYaw(AngleUnit.DEGREES);
        double error = heading - targetHeading;
        if (error > 180){
            error -= 360;
            heading -= 360;
        } else if (error < -180){
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
                drive(controller.calculate(heading, targetHeading), leftX, leftY, heading, Constants.Drive.DRIVE_POWER_MODIFIER);
            } else {
                drive(0, leftX, leftY, heading, Constants.Drive.DRIVE_POWER_MODIFIER);
            }
            telemetry.addData("Mode: ", "PID");
        } else {
            drive(rightX, leftX, leftY, heading, Constants.Drive.DRIVE_POWER_MODIFIER);
            firstTimeAfterStickRelease = true;
            telemetry.addData("Mode: ", "Rotating");
        }


        telemetry.addData("Heading: ", heading);
        telemetry.addData("Error: ", error);
        telemetry.addData("Target: ", targetHeading);
        telemetry.addData("Max Error: ", maxError);

    }

    public void resetYaw(){
        imu.resetYaw();
        targetHeading = 0;
    }
}