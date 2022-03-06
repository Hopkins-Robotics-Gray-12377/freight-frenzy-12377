package org.firstinspires.ftc.teamcode.AutoTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.VisionTests.GobiidaeVision.Gobiidae;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="AStatesLeftBlueAuto")
public class StatesLeftBlueAuto extends LinearOpMode {


    Pose2d startPose = new Pose2d(22, 62, Math.toRadians(90));

    //    Pose2d startDepositInter = new Pose2d(-25, -60, Math.toRadians(-90));
    Pose2d depositPose = new Pose2d(-1, 44, Math.toRadians(90));

    Pose2d duck1Pose = new Pose2d(-25, 60, Math.toRadians(90));
    Pose2d warehousePark1 = new Pose2d(9, 60, Math.toRadians(0));
    Pose2d warehousePark2 = new Pose2d(58, 58, Math.toRadians(0));



    public ElapsedTime launchTime = new ElapsedTime();

    enum RedDepot {
        Start,
        StartDepositInter,
        Deposit,
        StoragePark,
        WarehousePark1,
        WarehousePark2,
        MineralParkFinal,
        Duck1,
        Duck2,
        Carousel,
        Park
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Bot robo = new Bot();
        robo.autoInit(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);


        // Left Red
        Trajectory startDepositInter = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(9, 60, Math.toRadians(90)))
                .build();

        Trajectory Deposit = drive.trajectoryBuilder(new Pose2d(9, 60, Math.toRadians(90)))
                .lineToSplineHeading(depositPose)
                .build();

        Trajectory Warehouse1 = drive.trajectoryBuilder(depositPose)
                .lineToSplineHeading(warehousePark1)
                .build();

        Trajectory Warehouse2 = drive.trajectoryBuilder(warehousePark1)
                .lineToSplineHeading(warehousePark2)
                .build();



        Gobiidae pipeline;
        pipeline = new Gobiidae();
        pipeline.setup(hardwareMap);

        sleep(500);

        while (!opModeIsActive()) {

            telemetry.update();
        }


        ElapsedTime init = new ElapsedTime();

        init.reset();


        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {

            int results = pipeline.getAnalysis();

//            sleep(1000);

            RedDepot currState = RedDepot.Start;
            drive.followTrajectoryAsync(startDepositInter);

            while (opModeIsActive() && !isStopRequested()) {
                // Our state machine logic
                // You can have multiple switch statements running together for multiple state machines
                // in parallel. This is the basic idea for subsystems and commands.

                // We essentially define the flow of the state machine through this switch statement
                switch (currState) {
                    case Start:

                        if (!drive.isBusy()) {
                            // move slides to deposit item
                            drive.followTrajectoryAsync(Deposit);

                            currState = RedDepot.StartDepositInter;
                        }
                        break;

                    case StartDepositInter:

                        if (!drive.isBusy()) {
                            if (results == 1) {
                                robo.lowDeposit();
                            } else if (results == 2) {
                                robo.middleDeposit();
                            } else {
                                robo.highDeposit();
                            }

                            sleep(3000);

                            robo.autoLowerSlides();

                            sleep(2000);


                            currState = RedDepot.WarehousePark1;
                        }

                        break;

                    case WarehousePark1:
                        if (!drive.isBusy()) {
                            robo.autoRaiseSlidesALittle();

                            drive.followTrajectoryAsync(Warehouse1);

                            currState = RedDepot.WarehousePark2;
                        }

                        break;
                    case WarehousePark2:

                        if (!drive.isBusy()) {


                            drive.followTrajectoryAsync(Warehouse2);

                            currState = RedDepot.Park;
                        }

                        break;


                    case Park:

                        break;




                }

                // Anything outside of the switch statement will run independent of the currentState
                drive.update();

                telemetry.addData("State", currState);
                telemetry.update();
            }


        }
    }
}