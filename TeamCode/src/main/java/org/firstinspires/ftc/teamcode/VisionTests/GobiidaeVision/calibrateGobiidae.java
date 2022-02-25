package org.firstinspires.ftc.teamcode.VisionTests.GobiidaeVision;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.VisionTests.GobiidaeVision.Gobiidae;

@Autonomous(name="Calibrate Camera Threshold", group="Special")
public class calibrateGobiidae extends LinearOpMode {

    Gobiidae camera = new Gobiidae();


    @Override
    public void runOpMode(){
        camera.setup(hardwareMap);

        waitForStart();

        while(!isStopRequested()){
            camera.calibrateRoutine();
            telemetry.addData("Thres Calc ", camera.storage.STORED_THRESHOLD);
            telemetry.update();
            sleep(10);
        }
    }
}