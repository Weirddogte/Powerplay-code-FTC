package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Vector;

@Autonomous(name="Roadrunner Test", group="Linear Opmode")

public class Roadrunner_Test extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0,0,0);

        drive.setPoseEstimate(startPose);



        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(20,20), Math.toRadians(90))
                                .build();
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d())
                        .lineToConstantHeading(new Vector2d(1,0))
                                .build();
        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
                        .back(1)
                                .build();






        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);



    }
}
