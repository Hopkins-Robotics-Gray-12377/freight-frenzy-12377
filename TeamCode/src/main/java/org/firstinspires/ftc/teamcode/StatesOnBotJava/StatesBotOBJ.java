package org.firstinspires.ftc.teamcode.StatesOnBotJava;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


public class StatesBotOBJ {
    //////////////////
    /* DECLARATIONS */
    //////////////////

    // Control Hub - 173, now
    /*
    Control Hub

    Motors:
    0 - front left
    1 - front right
    2 - intake motor
    3 - carousel motor

    Servo:
    0 - holder for game pieces
    1 - armCapper servo

    Expansion Hub

    Motors:
    0 - back right
    1 - back left
    2 - slide motor

     */



    // constants for power, servo positions, etc.
    final double intakePower = .8;
    final double outtakePower = -1 * intakePower;

    final double holderPickup = .06;
    final double holderDump = 1;

    final double holderExtraLow = 0;


    final double carouselPower = .75;

    public int initialSlidePos = 0;
    public int topSlidePos = 0;
    public int lowSlidePos = 0;
    public int middleSlidePos = 0;

    final int lowSlideOffset = 2550; // tune
    final int middleSlideOffset = 2800; // tune
    final int highSlideOffset = 4200;


    //DRIVE//
    public DcMotor frontLeft   = null;
    public DcMotor backLeft    = null;
    public DcMotor backRight   = null;
    public DcMotor frontRight  = null;


    //Carousel//
    public DcMotor carousel    = null;

    //Intake//
    public DcMotor intake      = null;

    //Slides///
    public DcMotor slides     = null;

    //Holder//
    public Servo holder       = null;

    //Capper//
    public Servo capper       = null;


    //IMU//
    BNO055IMU imu;

    HardwareMap hwMap           =  null;

    /* Constructor */
    public void bot (){
    }


    /* Initialize standard Hardware interfaces */
    public void testInit(HardwareMap hwMap) {
        //////////////////////////////////
        /* RETRIEVING STUFF FROM PHONES */
        //////////////////////////////////


        //Carousel//
        carousel = hwMap.dcMotor.get("carousel");
        carousel.setDirection(DcMotor.Direction.FORWARD);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Intake//
        intake = hwMap.dcMotor.get("intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Slides//
        slides = hwMap.dcMotor.get("slides");
        slides.setDirection(DcMotor.Direction.FORWARD);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //holder//
        holder = hwMap.servo.get("holder");

        //IMU//
        imu = hwMap.get(BNO055IMU.class, "imu");
    }

    public void autoInit(HardwareMap hwMap) {
        //////////////////////////////////
        /* RETRIEVING STUFF FROM PHONES */
        //////////////////////////////////

        //DRIVE//
        SampleMecanumDrive drive = new SampleMecanumDrive(hwMap);

        //Carousel//
        carousel = hwMap.dcMotor.get("carousel");
        carousel.setDirection(DcMotor.Direction.FORWARD);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Intake//
        intake = hwMap.dcMotor.get("intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Slides//
        slides = hwMap.dcMotor.get("slides");
        slides.setDirection(DcMotor.Direction.REVERSE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        initialSlidePos = slides.getCurrentPosition();
        topSlidePos  = initialSlidePos + highSlideOffset;
        middleSlidePos = initialSlidePos + middleSlideOffset;
        lowSlidePos = initialSlidePos + lowSlideOffset;
//        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //holder//
        holder = hwMap.servo.get("holder");

        //capper//
        capper = hwMap.servo.get("armCapper");

        //IMU//
        imu = hwMap.get(BNO055IMU.class, "imu");
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        //////////////////////////////////
        /* RETRIEVING STUFF FROM PHONES */
        //////////////////////////////////

        //DRIVE//
        frontLeft   = hwMap.dcMotor.get("fl");
        backLeft    = hwMap.dcMotor.get("bl");
        backRight   = hwMap.dcMotor.get("br");
        frontRight  = hwMap.dcMotor.get("fr");
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Carousel//
        carousel = hwMap.dcMotor.get("carousel");
        carousel.setDirection(DcMotor.Direction.FORWARD);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Intake//
        intake = hwMap.dcMotor.get("intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Slides//
        slides = hwMap.dcMotor.get("slides");
        slides.setDirection(DcMotor.Direction.REVERSE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        initialSlidePos = slides.getCurrentPosition();
        topSlidePos  = initialSlidePos + highSlideOffset;
        middleSlidePos = initialSlidePos + middleSlideOffset;
        lowSlidePos = initialSlidePos + lowSlideOffset;
//        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //holder//
        holder = hwMap.servo.get("holder");

        //capper//
        capper = hwMap.servo.get("armCapper");

        //IMU//
        imu = hwMap.get(BNO055IMU.class, "imu");
    }



    public void intake() {
        intake.setPower(this.intakePower);
    }

    public void outtake() {
        intake.setPower(outtakePower);
    }

    public void holderDeposit() {
        holder.setPosition(.75);
    }

    public void holderNormal() {
        holder.setPosition(.07);
    }

    public void holderHold() {
        holder.setPosition(.35);
    }

    public void holderExtraLow() {
        holder.setPosition(holderExtraLow);
    }

    public void moveCarousel() {
        carousel.setPower(carouselPower);
    }

    public void reverseCarousel() {carousel.setPower(-carouselPower);}

//    public void joystickMoveCapper(Gamepad gamepad2) {
//
//        capper.setPosition(capper.getPosition() + (-gamepad2.left_stick_y)/1000);
//    }

    public void autoDepositSlides() {
        while (slides.getCurrentPosition() < topSlidePos) {
            if (slides.getCurrentPosition() > initialSlidePos + 1000) {
                holderHold();
            }
            slides.setPower(.4);
        }


        if (slides.getCurrentPosition() >= topSlidePos) {
            slides.setPower(0);
            holderDeposit();
        }
    }

    public void lowDeposit() {
        autoCustomDeposit(lowSlidePos);
    }

    public void middleDeposit() {
        autoCustomDeposit(middleSlidePos);
    }

    public void highDeposit() {
        autoCustomDeposit(topSlidePos);
    }

    private void autoCustomDeposit(int desiredPosition) {
        while (slides.getCurrentPosition() < desiredPosition) {
            if (slides.getCurrentPosition() > initialSlidePos + 1000) {
                holderHold();
            }
            slides.setPower(.4);
        }


        if (slides.getCurrentPosition() >= desiredPosition) {
            slides.setPower(0);
            holderDeposit();
        }
    }

    public void autoLowerSlides() {
        holderNormal();

        while (slides.getCurrentPosition() > initialSlidePos) {
            slides.setPower(-.1);
        }

        slides.setPower(0);
    }

    public void autoRaiseSlidesALittle() {
        holderNormal();

        while (slides.getCurrentPosition() < initialSlidePos + 200) {
            slides.setPower(.4);
        }

        slides.setPower(0);
    }

    public void autoLowerSlidesALittle() {
        holderNormal();
        holderNormal();

        while (slides.getCurrentPosition() > initialSlidePos + 200) {
            slides.setPower(-.1);
        }

        slides.setPower(0);
    }


    public void mecanumDrive(double pX, double pY, double pRot){
        frontLeft.setPower(pY + pRot);
        backLeft.setPower(pX - pRot);
        backRight.setPower(pY - pRot);
        frontRight.setPower(pX + pRot);
    }


    public void powerDrive(double power){
        frontLeft.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        frontRight.setPower(power);
    }


    public void resetDrive(){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void runToPosDrive(){
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void runWithoutEncoderDrive(){
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void imu(){
        /* IMU STUFF */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);
    }
}