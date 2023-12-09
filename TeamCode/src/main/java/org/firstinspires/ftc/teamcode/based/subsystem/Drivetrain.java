package org.firstinspires.ftc.teamcode.based.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.Algorithms;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.DcMotorWrapper;

import static org.firstinspires.ftc.teamcode.common.RobotHardwareConfig.Drive;

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
    private double heading;

    public static double kP = 0.015;
    public static double kI = 0;
    public static double kD = 0;
    public static double margin = 1.5;
    public static double maxAllowedError = 350;
    private double targetHeading;
    private boolean firstTimeAfterStickRelease;
    private PIDController controller;

    private double maxError;

    private State state;



    public Drivetrain(HardwareMap hwMap, DcMotor.ZeroPowerBehavior zeroPowerBehavior, DcMotor.RunMode runMode, Telemetry telemetry){
        super();
        this.telemetry = telemetry;
//        this.frontLeftMotor = hwMap.get(DcMotorEx.class, Drive.FL_STRING);
//        this.frontRightMotor = hwMap.get(DcMotorEx.class, Drive.FR_STRING);
//        this.backLeftMotor = hwMap.get(DcMotorEx.class, Drive.BL_STRING);
//        this.backRightMotor = hwMap.get(DcMotorEx.class, Drive.BR_STRING);

//        this.frontLeftMotor.setMode(runMode);
//        this.frontRightMotor.setMode(runMode);
//        this.backLeftMotor.setMode(runMode);
//        this.backRightMotor.setMode(runMode);

//        this.frontLeftMotor.setDirection(Drive.FL_DIRECTION);
//        this.frontRightMotor.setDirection(Drive.FR_DIRECTION);
//        this.backLeftMotor.setDirection(Drive.BL_DIRECTION);
//        this.backRightMotor.setDirection(Drive.BR_DIRECTION);

//        this.frontLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
//        this.frontRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
//        this.backLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
//        this.backRightMotor.setZeroPowerBehavior(zeroPowerBehavior);

        this.frontLeftMotor = new DcMotorWrapper(Drive.FL_STRING, hwMap, Drive.FL_DIRECTION, Drive.DEFAULT_RUNMODE);
        this.frontRightMotor = new DcMotorWrapper(Drive.FR_STRING, hwMap, Drive.FR_DIRECTION, Drive.DEFAULT_RUNMODE);
        this.backLeftMotor = new DcMotorWrapper(Drive.BL_STRING, hwMap, Drive.BL_DIRECTION, Drive.DEFAULT_RUNMODE);
        this.backRightMotor = new DcMotorWrapper(Drive.BR_STRING, hwMap, Drive.BR_DIRECTION, Drive.DEFAULT_RUNMODE);

        this.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.imu = hwMap.get(IMU.class, Drive.IMU_STRING);
        this.imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                Drive.IMU_LOGO_FACING,
                                Drive.IMU_USB_FACING
                        )
                )
        );

        this.controller = new PIDController(kP, kI, kD);
    }

    public Drivetrain(HardwareMap hwMap, DcMotor.ZeroPowerBehavior behavior, DcMotor.RunMode runMode){
        this(hwMap, behavior, runMode, null);
    }

    public Drivetrain(HardwareMap hwMap, DcMotor.ZeroPowerBehavior behavior){
        this(hwMap, behavior, Drive.DEFAULT_RUNMODE);
    }

    public Drivetrain(HardwareMap hwMap){
        this(hwMap, Drive.DEFAULT_BEHAVIOR);
    }

    public void resetYaw(){
        this.imu.resetYaw();
        this.targetHeading = 0;
    }

    public void driveRaw(double strafe, double forward, double rotation){
        double[] motorArraySpeeds = Algorithms.returnMecanumValues(forward, strafe, rotation, heading, Constants.Drive.DRIVE_POWER_MODIFIER);

        frontLeftMotor.setPower(motorArraySpeeds[Constants.Drive.MECANUM_FRONT_LEFT_MOTOR]);
        frontRightMotor.setPower(motorArraySpeeds[Constants.Drive.MECANUM_FRONT_RIGHT_MOTOR]);
        backLeftMotor.setPower(motorArraySpeeds[Constants.Drive.MECANUM_BACK_LEFT_MOTOR]);
        backRightMotor.setPower(motorArraySpeeds[Constants.Drive.MECANUM_BACK_RIGHT_MOTOR]);
    }

    public void drive(double strafe, double forward, double rotation){
        double calcRot;

        double error = this.heading - this.targetHeading;

        double m_heading = this.heading;

        if (Double.compare(error, 180d) > 0){
            error -= 360;
            m_heading -= 360;
        } else if (Double.compare(error, -180d) < 0){
            error += 360;
            m_heading += 360;
        }

        if (Math.abs(error) < margin){
            error = 0;
        }

        calcRot = controller.calculate(m_heading, targetHeading);

        switch (state){
            case PID:
                if (Double.compare(rotation, 0d) == 0){
                    driveRaw(strafe, forward, calcRot);
                    break;
                }
                state = State.ROTATE;
            case ROTATE:
                if (Double.compare(rotation, 0d) == 0){
                    targetHeading = this.heading;
                    drive(strafe, forward, 0d);
                    state = State.PID;
                    break;
                }
                driveRaw(strafe, forward, rotation);
                break;

        }

        telemetry.addData("Heading: ", this.heading);
        telemetry.addData("Corrected heading: ", m_heading);
        telemetry.addData("Error: ", error);
    }


    @Override
    public void periodic(){
        YawPitchRollAngles yawPitchRollAngles = this.imu.getRobotYawPitchRollAngles();
        this.heading = yawPitchRollAngles.getYaw(AngleUnit.DEGREES);
    }


}