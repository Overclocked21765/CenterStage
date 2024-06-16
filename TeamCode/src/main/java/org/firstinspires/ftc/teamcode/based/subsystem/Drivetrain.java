package org.firstinspires.ftc.teamcode.based.subsystem;

import static org.firstinspires.ftc.teamcode.common.util.Algorithms.returnMecanumValues;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.Constants;

import org.firstinspires.ftc.teamcode.common.RobotHardwareConfig;

import java.util.function.DoubleSupplier;

@Config
public class Drivetrain extends SubsystemBase{
    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;

    private IMU imu;
    private Telemetry telemetry;

    private DoubleSupplier leftX, leftY, rightX;

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

    States state;

    public void init(HardwareMap hwMap, DcMotor.ZeroPowerBehavior zeroPowerBehavior, Telemetry telemetry){
        this.telemetry = telemetry;

        frontLeftMotor = hwMap.get(DcMotorEx.class, RobotHardwareConfig.Drive.FL_STRING);
        frontRightMotor = hwMap.get(DcMotorEx.class, RobotHardwareConfig.Drive.FR_STRING);
        backLeftMotor = hwMap.get(DcMotorEx.class, RobotHardwareConfig.Drive.BL_STRING);
        backRightMotor = hwMap.get(DcMotorEx.class, RobotHardwareConfig.Drive.BR_STRING);

        frontLeftMotor.setMode(RobotHardwareConfig.Drive.DEFAULT_RUNMODE);
        frontRightMotor.setMode(RobotHardwareConfig.Drive.DEFAULT_RUNMODE);
        backLeftMotor.setMode(RobotHardwareConfig.Drive.DEFAULT_RUNMODE);
        backRightMotor.setMode(RobotHardwareConfig.Drive.DEFAULT_RUNMODE);

        frontLeftMotor.setDirection(RobotHardwareConfig.Drive.FL_DIRECTION);
        frontRightMotor.setDirection(RobotHardwareConfig.Drive.FR_DIRECTION);
        backLeftMotor.setDirection(RobotHardwareConfig.Drive.BL_DIRECTION);
        backRightMotor.setDirection(RobotHardwareConfig.Drive.BR_DIRECTION);

        frontLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        frontRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
        backLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        backRightMotor.setZeroPowerBehavior(zeroPowerBehavior);

        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RobotHardwareConfig.Drive.IMU_LOGO_FACING,
                                RobotHardwareConfig.Drive.IMU_USB_FACING
                        )
                )
        );
    }


    private void drive(double rotation, double strafe, double forwardPower, double heading, double scalar){
        double [] motorArraySpeeds = returnMecanumValues(rotation, strafe, forwardPower, heading, scalar);

        frontLeftMotor.setPower(motorArraySpeeds[0]);
        frontRightMotor.setPower(motorArraySpeeds[1]);
        backLeftMotor.setPower(motorArraySpeeds[2]);
        backRightMotor.setPower(motorArraySpeeds[3]);
    }

    public Drivetrain(double heading, HardwareMap hardwareMap, Telemetry telemetry, GamepadEx gamepad){
        super();
        targetHeading = heading;
        state = States.PID;
        firstTimeAfterStickRelease = false;
        controller = new PIDController(kP, kI, kD);
        maxError = 0;
        this.leftX = gamepad::getLeftX;
        this.leftY = gamepad::getLeftY;
        this.rightX = gamepad::getRightX;
        this.init(hardwareMap, DcMotor.ZeroPowerBehavior.FLOAT, telemetry);
        this.resetYaw();

    }

    /**
     *
     * @param leftX
     * @param leftY
     * @param rightX
     *
     * used for pid
     */
    public void pid(double leftX, double leftY, double rightX){
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
                drive(controller.calculate(heading, targetHeading), leftX, leftY, heading, Constants.ConstantsDrive.DRIVE_POWER_MODIFIER);
            } else {
                drive(0, leftX, leftY, heading, Constants.ConstantsDrive.DRIVE_POWER_MODIFIER);
            }
            telemetry.addData("Mode: ", "PID");
        } else {
            drive(rightX, leftX, leftY, heading, Constants.ConstantsDrive.DRIVE_POWER_MODIFIER);
            firstTimeAfterStickRelease = true;
            telemetry.addData("Mode: ", "Rotating");
        }


        telemetry.addData("Heading: ", heading);
        telemetry.addData("Error: ", error);
        telemetry.addData("Target: ", targetHeading);
        telemetry.addData("Max Error: ", maxError);

    }

    public void update(){
        this.pid(-leftX.getAsDouble(), -leftY.getAsDouble(), -rightX.getAsDouble());
    }


    public void resetYaw(){
        imu.resetYaw();
        targetHeading = 0;
    }
}