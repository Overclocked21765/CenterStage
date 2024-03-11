package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.based.command.FollowPathCommand;
import org.firstinspires.ftc.teamcode.based.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.based.subsystem.AutoDrivetrain;
import org.firstinspires.ftc.teamcode.based.subsystem.BulkReader;
import org.firstinspires.ftc.teamcode.based.subsystem.EndEffectorSubsystem;
import org.firstinspires.ftc.teamcode.based.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOp;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.nonEOCV.PropPipelineStreamable;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Red Right Auto")
@Config
public class RedRightAuto extends LinearOpMode {
    private BulkReader reader;
    private ArmSubsystem m_arm;
    private LiftSubsystem m_lift;
    private AutoDrivetrain drive;
    private EndEffectorSubsystem m_effector;
    private PropPipelineStreamable processor;
    private VisionPortal portal;

    private TrajectorySequence leftTrajectory;
    private TrajectorySequence middleTrajectory;
    private TrajectorySequence rightTrajectory;

    private TrajectorySequence leftBackdropTrajectory;
    private TrajectorySequence leftParkTrajectory;

    public static int WAIT_FOR_RELEASE_TIME = 500;


    public static Pose2d START_POSE = new Pose2d(12, -65, Math.toRadians(-90));

    public static Pose2d ZONE_3 = new Pose2d(52.6, -42.3, 0);
    public static Pose2d ZONE_2 = new Pose2d(52.6, -36.4, 0);
    public static Pose2d ZONE_1 = new Pose2d(52.6, -29.8, 0);

    public static Pose2d PARK = new Pose2d(52.6, -12, 0);

    public static Pose2d LEFT_SCORE = new Pose2d(5, -32, Math.toRadians(167.91));
    public static Pose2d LEFT_TRANSFER_POINT = new Pose2d(12.5, -44.33, Math.toRadians(92.68));

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        reader = new BulkReader(hardwareMap);

        m_arm = new ArmSubsystem(hardwareMap);
        m_lift = new LiftSubsystem(hardwareMap, telemetry);
        m_effector = new EndEffectorSubsystem(hardwareMap, telemetry);

        m_arm.setPositions(0.2, 0.2);
        m_effector.grab();


        drive = new AutoDrivetrain(hardwareMap);

        leftTrajectory = drive.trajectorySequenceBuilder(START_POSE)
                .setReversed(true)
                .splineTo(new Vector2d(LEFT_TRANSFER_POINT.getX(), LEFT_TRANSFER_POINT.getY()), LEFT_TRANSFER_POINT.getHeading())
                .splineTo(new Vector2d(LEFT_SCORE.getX(), LEFT_SCORE.getY()), LEFT_SCORE.getHeading())
                .build();

        leftBackdropTrajectory = drive.trajectorySequenceBuilder(leftTrajectory.end())
                .lineToLinearHeading(ZONE_1)
                .build();

        leftParkTrajectory = drive.trajectorySequenceBuilder(leftBackdropTrajectory.end())
                .lineToConstantHeading(new Vector2d(PARK.getX(), PARK.getY()))
                .turn(Math.toRadians(90))
                .build();

        drive.setPoseEstimate(START_POSE);



        waitForStart();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowPathCommand(drive, leftTrajectory),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> m_effector.releaseRight(), m_effector),
                                        new WaitCommand(WAIT_FOR_RELEASE_TIME),

                                        new SequentialCommandGroup(
                                                new InstantCommand(() -> m_arm.setDepositPosition(), m_arm),
                                                new InstantCommand(() -> m_effector.passThrough(), m_effector),
                                                new WaitCommand(TeleOp.timeReachDeposit),
                                                new InstantCommand(() -> m_effector.moveToDeposit(), m_effector),
                                                new InstantCommand(() -> m_lift.setTarget(300), m_lift)
                                        )
                                ),
                                new FollowPathCommand(drive, leftBackdropTrajectory)
                        ),
                        new InstantCommand(() -> m_effector.release()),
                        new WaitCommand(WAIT_FOR_RELEASE_TIME),
                        new FollowPathCommand(drive, leftParkTrajectory),
                        new InstantCommand(() -> m_lift.setTarget(Constants.Lift.GROUND_POSITION), m_lift)
                )
        );

        while (opModeIsActive() && !isStopRequested()){
            CommandScheduler.getInstance().run();
            telemetry.update();
        }

    }
}
