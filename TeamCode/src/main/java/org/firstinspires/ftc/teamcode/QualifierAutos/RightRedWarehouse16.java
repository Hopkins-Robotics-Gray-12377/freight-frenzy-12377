package org.firstinspires.ftc.teamcode.QualifierAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;


@Autonomous
public class RightRedWarehouse16 extends LinearOpMode {

//    Pose2d startPose = new Pose2d(-62.0, -18.0, 0.0);

//
//    private Pose2d startPose = new Pose2d(-62.5, 34.0, Math.toRadians(-90.0));
//
//    private Pose2d depositPose = new Pose2d(-25.0, -8.0, Math.toRadians(-90.0));
//
//    private Pose2d mineralPark1 = new Pose2d(-50.0, -10.0, Math.toRadians(-90.0));
//
//    private Pose2d mineralFinalPark = new Pose2d(-50.0, -50.0, Math.toRadians(-90.0));


    private Pose2d startPose = new Pose2d(-62.5, 34.0, 0.0);

    private Pose2d depositPose = new Pose2d(-25.0, -8.0, Math.toRadians(-90.0));

    private Pose2d mineralPark1 = new Pose2d(-50.0, -10.0, Math.toRadians(-90.0));

    private Pose2d mineralFinalPark = new Pose2d(-50.0, -50.0, Math.toRadians(-90.0));


    public ElapsedTime launchTime = new ElapsedTime();

    enum RightRed {
        Start,
        Duck,
        Deposit1,
        Deposit,
        StoragePark,
        MineralPark1,
        MineralPark2,
        MineralParkFinal,
        Park
    }


    RightRed currentLeftRedState = RightRed.Start;


    @Override
    public void runOpMode() throws InterruptedException{
        Bot robo = new Bot();
        robo.autoInit(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);


        // Right Red

        Trajectory deposit = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(depositPose)
                .build();

        Trajectory parkInMineral1 = drive.trajectoryBuilder(deposit.end())
                .lineToSplineHeading(mineralPark1)
                .build();

        Trajectory finalParkInMineral = drive.trajectoryBuilder(parkInMineral1.end())
                .forward(50)
                .build();


        ElapsedTime init = new ElapsedTime();

        init.reset();

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {
            currentLeftRedState = RightRed.Deposit;
            drive.followTrajectoryAsync(deposit);

            while (opModeIsActive() && !isStopRequested()) {
                // Our state machine logic
                // You can have multiple switch statements running together for multiple state machines
                // in parallel. This is the basic idea for subsystems and commands.

                // We essentially define the flow of the state machine through this switch statement
                switch (currentLeftRedState) {
                    case Deposit:

                        if (!drive.isBusy()) {
                            // move slides to deposit item
                            robo.autoDepositSlides();
                            sleep(1000);
                            currentLeftRedState = RightRed.MineralPark1;

                        }

                        break;


                    case MineralPark1:

                        if (!drive.isBusy()) {
                            robo.autoLowerSlides();
                            drive.followTrajectoryAsync(parkInMineral1);
                            currentLeftRedState = RightRed.MineralParkFinal;
                        }

                        break;

//
                    case MineralParkFinal:

                        if (!drive.isBusy()) {
                            drive.followTrajectoryAsync(finalParkInMineral);
                            currentLeftRedState = RightRed.Park;
                        }


                        break;

                    case Park:
                        break;


                }

                // Anything outside of the switch statement will run independent of the currentState
                drive.update();

                telemetry.addData("State", currentLeftRedState);
                telemetry.update();
            }


        }
    }

}