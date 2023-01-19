package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MecanumHardware {
    private LinearOpMode myOpMode = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private Servo intakeflip=null;
    private Servo claw=null;
    private DcMotor lift=null;
    private Servo coneflip=null;
    private DcMotor slider = null;


//    BNO055IMU imu;
//    Orientation angles;

    public ElapsedTime runtime = new ElapsedTime();

    public MecanumHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 20.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    public void init() {

//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.mode                = BNO055IMU.SensorMode.IMU;
//        parameters.loggingEnabled      = true;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//
//        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//        myOpMode.sleep(3000);

        leftBack = myOpMode.hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = myOpMode.hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = myOpMode.hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = myOpMode.hardwareMap.get(DcMotor.class, "rightFront");
        intakeflip = myOpMode.hardwareMap.get(Servo.class, "intakeflip");
        claw = myOpMode.hardwareMap.get(Servo.class, "claw");
        lift = myOpMode.hardwareMap.get(DcMotor.class, "lift");
        coneflip = myOpMode.hardwareMap.get(Servo.class, "coneflip");
        slider = myOpMode.hardwareMap.get(DcMotor.class, "slider");
        double speed;
        double strafe;
        double turn;
        int intakeliftarget=0;
        double liftpower;
        boolean clawpositon = true;

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeflip.setDirection(Servo.Direction.FORWARD);

        coneflip.setDirection(Servo.Direction.FORWARD);

        coneflip.setPosition(0.5);




        intakeflip.setPosition(0.115);
        claw.setPosition(1);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(200);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);

        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void part1(int numberofconesremaining){
        if(numberofconesremaining == 2){
            intakeflip.setPosition(0.115);
        } else if (numberofconesremaining == 1){
            intakeflip.setPosition(0.08);
        }
        claw.setPosition(0);

        myOpMode.sleep(450);
        intakeflip.setPosition(0.490);
        myOpMode.sleep(850);
        claw.setPosition(1);
        myOpMode.sleep(325);
        intakeflip.setPosition(0.1);
    }
    public void part2(){
        slider.setTargetPosition(700);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(1);
        while(slider.isBusy()){
            myOpMode.telemetry.update();

        }
        myOpMode.telemetry.addData("Done","DOne");
        myOpMode.telemetry.update();
        slider.setPower(0);
        coneflip.setPosition(0);
        myOpMode.sleep(735);
        coneflip.setPosition(0.5);

//        coneflip.setPower(1);
//        myOpMode.sleep(650);
//        coneflip.setPower(0);
//        myOpMode.sleep(100);
//        coneflip.setPower(-1);
//        myOpMode.sleep(575);
//        coneflip.setPower(0);
        slider.setTargetPosition(0);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(1);
        intakeflip.setPosition(0.08);
        while(slider.isBusy()){
            myOpMode.telemetry.addData("data",slider.getCurrentPosition());
            myOpMode.telemetry.addData("data2",claw.getPosition());
            myOpMode.telemetry.update();


            if(claw.getPosition() ==1){
                claw.setPosition(0);
                myOpMode.sleep(750);
                intakeflip.setPosition(0.490);
            }




        }
//
        claw.setPosition(1);
        myOpMode.sleep(325);
        intakeflip.setPosition(0.08);
        myOpMode.telemetry.addData("Done","DOne");
        myOpMode.telemetry.update();
    }

    public void flipspeed(int reps){
        part1(2);
        for (int i=0;i<=reps;i++)
        {
            part2();
        }


    }

    /*    public void ForwardDistance(double Power,int distance) {
            if (Power > 1.0){
                Power = 1.0;
            }
            else if (Power < 1.0){
                Power = -1.0;
            }
            int Distance = (int) (distance/3*3.14)*560;
            leftBack.setTargetPosition(Distance);
            leftFront.setTargetPosition(Distance);
            rightBack.setTargetPosition(Distance);
            rightFront.setTargetPosition(Distance);
            leftFront.setPower(Power);
            rightFront.setPower(Power);
            leftBack.setPower(Power);
            rightBack.setPower(Power);
            while (myOpMode.opModeIsActive() && leftBack.isBusy()){
                telemetry.update();
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);*/
    public void ForwardDistance(double Power, int distance, double diameter, int gear_reduction) {

//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);


        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int Distance = leftBack.getCurrentPosition() + (int) ((distance / (diameter * 3.14)) * 537.6);

        leftBack.setTargetPosition(Distance);
        leftFront.setTargetPosition(Distance);
        rightBack.setTargetPosition(Distance);
        rightFront.setTargetPosition(Distance);

        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBack.setPower(Math.abs(Power));
        leftFront.setPower(Math.abs(Power));
        rightBack.setPower(Math.abs(Power));
        rightFront.setPower(Math.abs(Power));
        runtime.reset();
        while (leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {

//            if (angles.firstAngle < -5){
//                leftBack.setPower(Math.abs(Power)+0.05);
//                leftFront.setPower(Math.abs(Power)+0.05);
//                rightBack.setPower(Math.abs(Power));
//                rightFront.setPower(Math.abs(Power));
//                myOpMode.telemetry.addData("Motor Powers", " %7d :%7d :%7d :%7d", leftBack.getPower(), leftFront.getPower(),rightBack.getPower(), rightFront.getPower());
//                myOpMode.telemetry.update();
//            } else if (angles.firstAngle > 5) {
//                leftBack.setPower(Math.abs(Power));
//                leftFront.setPower(Math.abs(Power));
//                rightBack.setPower(Math.abs(Power)+0.05);
//                rightFront.setPower(Math.abs(Power)+0.05);
//                myOpMode.telemetry.addData("Motor Powers", " %7d :%7d :%7d :%7d", leftBack.getPower(), leftFront.getPower(),rightBack.getPower(), rightFront.getPower());
//                myOpMode.telemetry.update();
//            } else {
//                leftBack.setPower(Math.abs(Power));
//                leftFront.setPower(Math.abs(Power));
//                rightBack.setPower(Math.abs(Power));
//                rightFront.setPower(Math.abs(Power));
//                myOpMode.telemetry.addData("Motor Powers", " %7d :%7d :%7d :%7d", leftBack.getPower(), leftFront.getPower(),rightBack.getPower(), rightFront.getPower());
//                myOpMode.telemetry.update();
//            }

            // Display it for the driver.
            myOpMode.telemetry.addData("Running to", " %7d :%7d", Distance, Distance);
            myOpMode.telemetry.addData("Currently at", " at %7d :%7d", leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
            myOpMode.telemetry.update();
            myOpMode.sleep(1);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);


    }

    public void ForwardTime(double Power, long Time) {
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(Power);
        rightFront.setPower(Power);
        leftBack.setPower(Power);
        rightBack.setPower(Power);
        myOpMode.sleep(Time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void sideTime(double Power, long Time) {
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(Power);
        rightFront.setPower(-Power);
        leftBack.setPower(-Power);
        rightBack.setPower(Power);
        myOpMode.sleep(Time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void sideDistance(double Power, int distance, double diameter, int gear_reduction) {
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int Distance = leftBack.getCurrentPosition() + (int) ((distance / (diameter * 3.14)) * 537.6);

        leftBack.setTargetPosition(-Distance);
        leftFront.setTargetPosition(Distance);
        rightBack.setTargetPosition(Distance);
        rightFront.setTargetPosition(-Distance);

        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(Power);
        rightFront.setPower(-Power);
        leftBack.setPower(-Power);
        rightBack.setPower(Power);
        runtime.reset();
        while (leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {

            // Display it for the driver.
            myOpMode.telemetry.addData("Running to", " %7d :%7d", Distance, Distance);
            myOpMode.telemetry.addData("Currently at", " at %7d :%7d", leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
            myOpMode.telemetry.update();
            myOpMode.sleep(1);
        }
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

    }
}
