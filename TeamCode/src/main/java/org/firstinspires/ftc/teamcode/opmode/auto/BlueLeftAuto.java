package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.based.command.FollowPathCommand;
import org.firstinspires.ftc.teamcode.based.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.based.subsystem.AutoDrivetrain;
import org.firstinspires.ftc.teamcode.based.subsystem.BulkReader;
import org.firstinspires.ftc.teamcode.based.subsystem.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.based.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.util.LoopTimer;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOp;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.nonEOCV.PropPipelineStreamable;
import org.firstinspires.ftc.teamcode.vision.pipelineEOCV.PropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Blue Left Auto")
@Config
public class BlueLeftAuto extends LinearOpMode {
    private BulkReader reader;
    private ArmSubsystem m_arm;
    private LiftSubsystem m_lift;
    private AutoDrivetrain drive;
    private EndEffectorSubsystem m_effector;
    private PropPipelineStreamable processor;
    private VisionPortal portal;

    private LoopTimer loopTimer;

    private TrajectorySequence leftTrajectory;
    private TrajectorySequence middleTrajectory;
    private TrajectorySequence rightTrajectory;

    private TrajectorySequence selectedTrajectoryStart;
    private TrajectorySequence selectedTrajectoryBackdrop;
    private TrajectorySequence parkTrajectory;

    private PropPipeline.Location propLocation = PropPipeline.Location.LEFT;

    private TrajectorySequence leftBackdropTrajectory;
    private TrajectorySequence leftParkTrajectory;

    private TrajectorySequence middleBackdropTrajectory;
    private TrajectorySequence middleParkTrajectory;

    private TrajectorySequence rightBackdropTrajectory;
    private TrajectorySequence rightParkTrajectory;



    public static int WAIT_FOR_RELEASE_TIME = 500;
    public static int AUTO_HEIGHT = 400;


    public static Pose2d START_POSE = new Pose2d(12, 65, Math.toRadians(90));

    public static Pose2d ZONE_1 = new Pose2d(53, 42.3, 0);
    public static Pose2d ZONE_2 = new Pose2d(53, 36.4, 0);
    public static Pose2d ZONE_3 = new Pose2d(53, 29.8, 0);

    public static Pose2d PARK = new Pose2d(52.6, 12, 0);

    public static Pose2d LEFT_SCORE = new Pose2d(12, 32, Math.toRadians(0));
    public static Pose2d MID_SCORE = new Pose2d(12, 36, Math.toRadians(90));
    public static Pose2d RIGHT_SCORE = new Pose2d(22.3, 41, Math.toRadians(90));

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        this.propLocation = PropPipeline.Location.LEFT;

        reader = new BulkReader(hardwareMap);

        m_arm = new ArmSubsystem(hardwareMap);
        m_lift = new LiftSubsystem(hardwareMap, telemetry);
        m_effector = new EndEffectorSubsystem(hardwareMap, telemetry);

        processor = new PropPipelineStreamable(telemetry, PropPipeline.Location.BLUE);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(processor)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(processor, 0);


        m_arm.setPositions(0.2, 0.2);
        m_effector.grab();


        drive = new AutoDrivetrain(hardwareMap);

        leftTrajectory = drive.trajectorySequenceBuilder(START_POSE)
                .lineToLinearHeading(LEFT_SCORE)
                .build();

        leftBackdropTrajectory = drive.trajectorySequenceBuilder(leftTrajectory.end())
                .lineToLinearHeading(ZONE_1)
                .build();

        leftParkTrajectory = drive.trajectorySequenceBuilder(leftBackdropTrajectory.end())
                .lineToLinearHeading(PARK)
                .turn(Math.toRadians(-90))
                .build();




        middleTrajectory = drive.trajectorySequenceBuilder(START_POSE)
                .lineToLinearHeading(MID_SCORE)
                .build();

        middleBackdropTrajectory = drive.trajectorySequenceBuilder(middleTrajectory.end())
                        .lineToLinearHeading(ZONE_2)
                                .build();

        middleParkTrajectory = drive.trajectorySequenceBuilder(middleBackdropTrajectory.end())
                .lineToLinearHeading(PARK)
                .turn(Math.toRadians(-90))
                .build();



        rightTrajectory = drive.trajectorySequenceBuilder(START_POSE)
                        .lineToLinearHeading(RIGHT_SCORE)
                                .build();

        rightBackdropTrajectory = drive.trajectorySequenceBuilder(rightTrajectory.end())
                                .lineToLinearHeading(ZONE_3)
                                        .build();

        rightParkTrajectory = drive.trajectorySequenceBuilder(rightBackdropTrajectory.end())
                .lineToLinearHeading(PARK)
                .turn(Math.toRadians(-90))
                .build();



        drive.setPoseEstimate(START_POSE);

        m_effector.moveToDeposit();


        while (opModeInInit()){
            m_lift.periodic();
            this.propLocation = processor.getLocation();
            telemetry.addData("location: ", propLocation);
            telemetry.update();
        }

        m_effector.moveToIntake();

        portal.stopLiveView();
        portal.stopStreaming();

        switch (propLocation){
            case RIGHT:
                selectedTrajectoryBackdrop = leftBackdropTrajectory;
                selectedTrajectoryStart = leftTrajectory;
                parkTrajectory = leftParkTrajectory;
                break;
            case CENTER:
                selectedTrajectoryBackdrop = middleBackdropTrajectory;
                selectedTrajectoryStart = middleTrajectory;
                parkTrajectory = middleParkTrajectory;
                break;
            case LEFT:
                selectedTrajectoryBackdrop = rightBackdropTrajectory;
                selectedTrajectoryStart = rightTrajectory;
                parkTrajectory = rightParkTrajectory;
                break;
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowPathCommand(drive, selectedTrajectoryStart),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> m_effector.releaseRight(), m_effector),
                                        new WaitCommand(WAIT_FOR_RELEASE_TIME),

                                        new SequentialCommandGroup(
                                                new InstantCommand(() -> m_arm.setDepositPosition(), m_arm),
                                                new InstantCommand(() -> m_effector.passThrough(), m_effector),
                                                new WaitCommand(TeleOp.timeReachDeposit + 500),
                                                new InstantCommand(() -> m_effector.moveToDeposit(), m_effector),
                                                new InstantCommand(() -> m_lift.setTarget(Constants.ConstantsLift.AUTO_BACKDROP_HEIGHT), m_lift)
                                        )
                                ),
                                new FollowPathCommand(drive, selectedTrajectoryBackdrop)
                        ),
                        new InstantCommand(() -> m_effector.release()),
                        new WaitCommand(WAIT_FOR_RELEASE_TIME),
                        new InstantCommand(() -> m_lift.setTarget(Constants.ConstantsLift.GROUND_POSITION)),
                        new WaitCommand(Constants.ConstantsAuto.TIME_WAIT_BEFORE_MOVE),

                        new FollowPathCommand(drive, parkTrajectory),
                        new SequentialCommandGroup(
                                new FunctionalCommand(
                                        () -> m_lift.setTarget(Constants.ConstantsLift.GROUND_POSITION),
                                        () -> {},
                                        (interrupted) -> {},
                                        () -> m_lift.getCurrentPosition() <= Constants.ConstantsLift.RED_ZONE,
                                        m_lift
                                ),
                                new InstantCommand(() -> m_effector.passThrough(), m_effector),
                                new WaitCommand(TeleOp.timeWristStraighten),
                                new InstantCommand(() -> m_effector.grab(), m_effector),
                                new InstantCommand(() -> m_arm.setPositions(0.4, 0.4), m_arm),
                                new WaitCommand(TeleOp.timeDepositToEffector),
                                new InstantCommand(() -> m_effector.moveToIntake(), m_effector),
                                new WaitCommand(TeleOp.timeEffectorTo02),
                                new InstantCommand(() -> m_arm.setIntakePosition(), m_arm)
                        )
                )
        );

        loopTimer = new LoopTimer(telemetry);

        while (opModeIsActive() && !isStopRequested()){
            CommandScheduler.getInstance().run();
            loopTimer.updateLoop();
            telemetry.update();
        }
        loopTimer.endLoop();
    }
}
