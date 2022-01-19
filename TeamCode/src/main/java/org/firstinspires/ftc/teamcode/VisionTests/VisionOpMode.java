package org.firstinspires.ftc.teamcode.VisionTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.ToggleMap;
import org.firstinspires.ftc.teamcode.UseMap;

@TeleOp(name="VisionTest", group="main")
public class VisionOpMode extends LinearOpMode {
    Bot robo = new Bot();


    private ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime launchTime = new ElapsedTime();


    @Override
    public void runOpMode() {

        Gobildae pipeline;
        pipeline = new Gobildae();
        pipeline.setup(hardwareMap);





        while (!opModeIsActive()) {

            telemetry.update();
        }


        waitForStart();


        while (opModeIsActive()) {


            int results = pipeline.getAnalysisSimple();

            if (results == 3)
                telemetry.addData("Position", "3");
            else if (results == 2)
                telemetry.addData("Position", "2");
            else if (results == 1)
                telemetry.addData("Position", "1");
            else
                telemetry.addData("Position", "0");

            telemetry.update();
        }
    }

}