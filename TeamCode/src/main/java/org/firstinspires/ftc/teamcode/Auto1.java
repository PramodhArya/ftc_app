package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@Autonomous(name="Autonomous Crater", group="Pushbot")
public class Auto1 extends LinearOpMode {

    public Timer Diego;
    public TimerTask task;

    /* Declare OpMode members. */
//    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device

    static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    int COUNTS_PER_REVOLUTION = 1120;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.5;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.7;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    static final double SERVO_DOWN = 30/280.0;
    static final double SERVO_UP = 180/280.0;

    boolean gold = false;

    ElapsedTime timer = new ElapsedTime();

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    ElapsedTime runtime = new ElapsedTime();

    private DcMotor front = null;
    private DcMotor right = null;
    private DcMotor bottom = null;
    private DcMotor left = null;
    private DcMotor lifter = null;
    private DcMotor flipper = null;

    private Servo winch = null;
    private Servo tilt = null;
    private CRServo intake = null;

    private enum goldMineral { LEFT, MIDDLE, RIGHT; }
    goldMineral goldMineralPosition = goldMineral.MIDDLE;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AaccPCv/////AAAAGUHMoe4QMEQnovLFMgFScJEVe3Zia3YYTEl3U1EXcd1XRE7aV9ONZFR91dfsSQ4tnOBYK10+SvF1S1LEGkjeQWDBFvKci3ki3K7E440/ZRB0YGIuxYUVrp9AZ0PtSExOtE6bFXyksrCPcD6IV8rHvNJYwWbE/LUqrCj18TtN0QbWBXfVSmXmRnfVWBDOA8O8v7kCZzeBm328KlYb105Uo48MICRipR9/oua0rJ1QNIY+ytwxHabLCZgNlMr64+In/xB3aCtnHjTC8ClSxirmschtzlq+up6CzYkahlX45SnV6mGqJ345uPzUnJzAr9Z6QDEd17veQPMP3zLheBDWM3l/590e+i5qFLvjVrcu/Njs\n";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        //*************************** IMU Setup ************************/

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addLine("setup finished");
        telemetry.update();

        //*************************** Hardware Maps ************************/

        front = hardwareMap.get(DcMotor.class, "front");
        right = hardwareMap.get(DcMotor.class, "right");
        bottom = hardwareMap.get(DcMotor.class, "bottom");
        left = hardwareMap.get(DcMotor.class, "left");
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        flipper = hardwareMap.get(DcMotor.class, "flipper");

//        front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        winch = hardwareMap.get(Servo.class, "winch");
        tilt = hardwareMap.get(Servo.class, "tilt");
        intake = hardwareMap.get(CRServo.class, "intake");

        front.setDirection(REVERSE);
        right.setDirection(REVERSE);

        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }

        waitForStart();
        tfod.shutdown();
        startUpdates();

        long start = runtime.now(TimeUnit.SECONDS);
        while (opModeIsActive() && (runtime.now(TimeUnit.SECONDS) < start + 5)) {
            colorCheck();
        }

        telemetry.addLine("finished color check");
        telemetry.update();

//        stopUpdates();

//        runtime.reset();

        unhook(15000); //1500
        moveY(DRIVE_SPEED, inchesToCounts(7), 5);
        moveX(DRIVE_SPEED, inchesToCounts(14), 5);
        sampleMineral();
        tilt.setPosition(SERVO_DOWN);

//        moveY(DRIVE_SPEED, -inchesToCounts(10), 5);
//        turnRelative(90);
//        long start = timer.now(TimeUnit.SECONDS);
//        while (timer.now(TimeUnit.SECONDS) - 2 < start && opModeIsActive()) {
//
//        }
//        turnRelative(90);
//        float start = timer.now(TimeUnit.SECONDS);
//        while (timer.now(TimeUnit.SECONDS) - 2 < start && opModeIsActive()) {
//
//        }
//        turnRelative(-90);
//        start = timer.now(TimeUnit.SECONDS);
//        while (timer.now(TimeUnit.SECONDS) - 2 < start && opModeIsActive()) {
//
//        }
//        turnRelative(90);
//        start = timer.now(TimeUnit.SECONDS);
//        while (timer.now(TimeUnit.SECONDS) - 2 < start && opModeIsActive()) {
//
//        }
//        turnDiego(90);
//        moveY(DRIVE_SPEED, inchesToCounts(10), 5);
//        turnDiego(90);
//        moveY(DRIVE_SPEED, inchesToCounts(10), 5);
//        turnDiego(180);
//        moveY(DRIVE_SPEED, inchesToCounts(10), 5);
//        turnDiego(270);
//        moveY(DRIVE_SPEED, inchesToCounts(10), 5);
//        gyroTurn(DRIVE_SPEED, -20);
//        pause();
//        moveY(DRIVE_SPEED,1000);
//        pause();
//        gyroTurn(TURN_SPEED, -4);
//        pause();
//        while (opModeIsActive()) {
////            telemetry.addData("front position", front.getCurrentPosition());
////            telemetry.addData("bottom position", bottom.getCurrentPosition());
////            telemetry.addData("front target", front.getTargetPosition());
////            telemetry.addData("bottom target", bottom.getTargetPosition());
//            telemetry.addLine("finished");
//            telemetry.update();
//        }

        stopUpdates();
        telemetry.addLine("finished");
        telemetry.update();
    }

    public void moveX(double power, int distance, long timeout) {
        long start = runtime.now(TimeUnit.SECONDS);
        bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottom.setTargetPosition(bottom.getCurrentPosition() + distance);
        front.setTargetPosition(front.getCurrentPosition() + distance);
        bottom.setPower(power);
        front.setPower(power);
        while (opModeIsActive() && front.isBusy() && bottom.isBusy() && runtime.now(TimeUnit.SECONDS) - timeout <= start) {
            telemetry.addData("front position", front.getCurrentPosition());
            telemetry.addData("bottom position", bottom.getCurrentPosition());
            telemetry.addData("front target", front.getTargetPosition());
            telemetry.addData("bottom target", bottom.getTargetPosition());
            telemetry.addData("elapsed time", runtime.now(TimeUnit.SECONDS) - start);
            telemetry.update();
        }
    }

    public void moveY(double power, int distance, long timeout) {
        long start = runtime.now(TimeUnit.SECONDS);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setTargetPosition(right.getCurrentPosition() + distance);
        left.setTargetPosition(left.getCurrentPosition() + distance);
        right.setPower(power);
        left.setPower(power);
        while (opModeIsActive() && right.isBusy() && left.isBusy() && runtime.now(TimeUnit.SECONDS) - timeout <= start) {
            telemetry.addData("right position", right.getCurrentPosition());
            telemetry.addData("left position", left.getCurrentPosition());
            telemetry.addData("right target", right.getTargetPosition());
            telemetry.addData("left target", left.getTargetPosition());
            telemetry.addData("elapsed time", runtime.now(TimeUnit.SECONDS) - start);
            telemetry.update();
        }
    }

    public void outtake(double time) {
        timer.reset();
        while(opModeIsActive() && (timer.time() < time)) {
            intake.setPower(1);
        }
        intake.setPower(0);
    }

    public void extend(double position) {
        winch.setPosition(position);
    }

    public void colorCheck() {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 2) {
                            boolean middleisGold = updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL);
                            boolean rightisGold = updatedRecognitions.get(1).getLabel().equals(LABEL_GOLD_MINERAL);
                            if (middleisGold) {
                                telemetry.addData("Gold Mineral Position", "Middle");
                                goldMineralPosition = goldMineral.MIDDLE;
                            } else if (rightisGold) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                goldMineralPosition = goldMineral.RIGHT;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Left");
                                goldMineralPosition = goldMineral.LEFT;
                            }
                        }
                        telemetry.update();
                    }
                }
    }

    public void sampleMineral() {
        switch (goldMineralPosition)
        {
            case LEFT:
                moveY(DRIVE_SPEED,inchesToCounts(-27),10);
                moveX(DRIVE_SPEED, inchesToCounts(56),10);
//                turnRelative(100);
                telemetry.addData("Gold mineral is on the","left");
                break;
            case MIDDLE:
                moveY(DRIVE_SPEED,inchesToCounts(-14), 5);
                moveX(DRIVE_SPEED,inchesToCounts(56),10);
//                turnRelative(155);
                telemetry.addData("Gold mineral is on the","middle");
                break;
            case RIGHT:
                moveY(DRIVE_SPEED,inchesToCounts(7),5);
                moveX(DRIVE_SPEED,inchesToCounts(36),5);
//                turnRelative(145);
//                moveY(DRIVE_SPEED, -inchesToCounts(30), 7);
//                turnRelative(100);
//                long start = timer.now(TimeUnit.SECONDS);
//                while (timer.now(TimeUnit.SECONDS) - 10 < start && opModeIsActive()) {
//
//                }

                telemetry.addData("Gold mineral is on the","right");
                break;
        }
    }

    public void unhook(int value) {
        lifter.setTargetPosition(value);
        lifter.setPower(1);
        while (opModeIsActive() && lifter.isBusy()) {
            telemetry.addData("Lifter", lifter.getCurrentPosition());
            telemetry.addData("lifter target", lifter.getTargetPosition());
            telemetry.update();
//            idle();
        }
        lifter.setPower(0);
    }

    public int inchesToCounts(double inches) {

        return (int)(inches * COUNTS_PER_REVOLUTION / (Math.PI * WHEEL_DIAMETER_INCHES));

    }

    public void turnAbsolute(float angle) {

        telemetry.addLine("Working");
        telemetry.update();

        float marginOfError = 12;

        float right = angle - marginOfError;
        float left = angle + marginOfError;

//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while (angles == null && opModeIsActive()) {

        }

        float startingAngle = angles.firstAngle;

        int direction = 0;

        if (angle - startingAngle > 180)
            direction = 1;
        else
            direction = -1;

        telemetry.addLine("Diego");
        telemetry.update();

        float multiplier = angle / Math.abs(angle);

        while (!(angles.firstAngle < left && angles.firstAngle > right) && opModeIsActive()) {
            front.setPower(0.35f * multiplier * direction);
            bottom.setPower(-0.35f * multiplier * direction);
            telemetry.addData("first Angle", angles.firstAngle);
            telemetry.addData("left angle", left);
            telemetry.addData("right angle", right);
            telemetry.addData("Within margin", (left <= angles.firstAngle || right >= angles.firstAngle));
            telemetry.addData("test", !(angles.firstAngle >= left || angles.firstAngle <= right));
            telemetry.update();
            //(left <= angles.firstAngle || right >= angles.firstAngle)
        }

        front.setPower(0);
        bottom.setPower(0);
    }
    public void startUpdates() {
        Diego = new Timer();
        task = new TimerTask() {
            @Override
            public void run() {
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        };

        Diego.scheduleAtFixedRate(task, 0, 1);
    }

    public void stopUpdates() {
        Diego.cancel();
    }

    class Range {

        private float low;
        private float high;

        public Range(float low, float high) {

            this.low = low;
            this.high = high;

        }

        public boolean contains(float number) {

            return (number >= low && number <= high);

        }

    }

    public void turnDiego(float angle) {

        front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        float margin = 3;
        float speed = 0.7f;

        while (angles == null && opModeIsActive()) {}

        float startingAngle = angles.firstAngle;

        float leftMargin = (angle - margin + 360) % 360;
        float rightMargin = (angle + margin + 360) % 360;

        Range rangeOne;
        Range rangeTwo;

        if (leftMargin > rightMargin) {

            rangeOne = new Range(leftMargin, rightMargin + 360);
            rangeTwo = new Range(leftMargin - 360, rightMargin);

        } else {

            rangeOne = new Range(leftMargin, rightMargin);
            rangeTwo = new Range(leftMargin, rightMargin);

        }

        int direction = 1;

        if (Math.abs(angle - startingAngle) > 360 - Math.abs(angle - startingAngle)) {
            direction = -1;
        }

        while (!(rangeOne.contains(angles.firstAngle + 180) || rangeTwo.contains(angles.firstAngle + 180)) && opModeIsActive()) {

            front.setPower(-speed * direction);
            right.setPower(-speed * direction);
            bottom.setPower(speed * direction);
            left.setPower(speed * direction);
            telemetry.addData("first Angle", angles.firstAngle + 180);
            telemetry.addData("left angle", leftMargin);
            telemetry.addData("right angle", rightMargin);
            telemetry.addData("Within margin", (leftMargin <= angles.firstAngle || rightMargin >= angles.firstAngle));
            telemetry.addData("test", (rangeOne.contains(angles.firstAngle + 180) || rangeTwo.contains(angles.firstAngle + 180)));
            telemetry.update();

        }

        front.setPower(0);
        right.setPower(0);
        bottom.setPower(0);
        left.setPower(0);

    }

    public void turnRelative(float angle) {

        while (angles == null && opModeIsActive()) {}

        turnDiego((angles.firstAngle + angle + 360)%360);

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}