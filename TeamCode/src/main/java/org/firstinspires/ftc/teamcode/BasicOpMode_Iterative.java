/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import java.util.concurrent.TimeUnit;
import java.util.Timer;
import java.util.TimerTask;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@Autonomous(name="Iterative Auto", group="Iterative Opmode")
@Disabled
public class BasicOpMode_Iterative extends OpMode
{

    public Timer Diego;
    public TimerTask task;

    boolean stopRequested;
    boolean isStarted;

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

    ElapsedTime timer = new ElapsedTime();

    BNO055IMU imu;

    Orientation angles;

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
    goldMineral goldMineralPosition = goldMineral.RIGHT;

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
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void init() {

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

        winch = hardwareMap.get(Servo.class, "winch");
        tilt = hardwareMap.get(Servo.class, "tilt");
        intake = hardwareMap.get(CRServo.class, "intake");

        front.setDirection(REVERSE);
        right.setDirection(REVERSE);

        stopRequested = false;
        isStarted = false;

        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        initVuforia();

        startUpdates();
    }

    @Override
    public void init_loop() {
//        colorCheck();
    }

    @Override
    public void start() {
        isStarted = true;
//        tfod.shutdown();
        turnDiego(90);
        long start = timer.now(TimeUnit.SECONDS);
        timer.reset();
        while (timer.now(TimeUnit.SECONDS) < (start + 5) && opModeIsActive()) {
            telemetry.addData("Time",timer.now(TimeUnit.SECONDS));
            telemetry.update();
        }
//        turnDiego(90);
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        stopRequested = true;
        stopUpdates();
    }

    public void moveX(double power, int distance, long timeout) {
        long start = runtime.now(TimeUnit.SECONDS);
        bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottom.setTargetPosition(bottom.getCurrentPosition() + distance);
        front.setTargetPosition(front.getCurrentPosition() + distance);
        bottom.setPower(power);
        front.setPower(power);
        while (front.isBusy() && bottom.isBusy() && runtime.now(TimeUnit.SECONDS) - timeout <= start) {
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
        while (right.isBusy() && left.isBusy() && runtime.now(TimeUnit.SECONDS) - timeout <= start) {
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
        while((timer.time() < time)) {
            intake.setPower(1);
        }
        intake.setPower(0);
    }

    public void extend(double position) {
        winch.setPosition(position);
    }

    public void colorCheck() {
        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }

//        while (true) {
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
//        }
//
//        if (tfod != null) {
//            tfod.shutdown();
//        }
    }

    public void sampleMineral() {
        switch (goldMineralPosition)
        {
            case LEFT:
                moveY(DRIVE_SPEED,inchesToCounts(-24),10);
                telemetry.addData("Gold mineral is on the","left");
                break;
            case MIDDLE:
                moveY(DRIVE_SPEED,inchesToCounts(-7), 5);
                moveX(DRIVE_SPEED,inchesToCounts(50),10);
                telemetry.addData("Gold mineral is on the","middle");
                break;
            case RIGHT:
                moveY(DRIVE_SPEED,inchesToCounts(10),5);
                telemetry.addData("Gold mineral is on the","right");
                break;
        }
    }

    public void unhook(int value) {
        lifter.setTargetPosition(value);
        lifter.setPower(1);
        while (lifter.isBusy()) {
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

        while (angles == null) {

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

        while (!(angles.firstAngle < left && angles.firstAngle > right)) {
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


        float margin = 10;
        float speed = 0.7f;

        while (angles == null) {}

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

        if (Math.abs(angle - startingAngle) > 180) {
            direction = -1;
        }

        while (!(rangeOne.contains(angles.firstAngle + 180) || rangeTwo.contains(angles.firstAngle + 180))) {

            front.setPower(speed * direction);
            right.setPower(speed * direction);
            bottom.setPower(-speed * direction);
            left.setPower(-speed * direction);
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

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public final boolean opModeIsActive() {
        boolean isActive = !this.isStopRequested() && this.isStarted();
        if (isActive) {
            idle();
        }
        return isActive;
    }

    public final boolean isStopRequested() {
        return this.stopRequested || Thread.currentThread().isInterrupted();
    }

    public final void idle() {
        // Otherwise, yield back our thread scheduling quantum and give other threads at
        // our priority level a chance to run
        Thread.yield();
    }

    public final boolean isStarted() {
        return this.isStarted || Thread.currentThread().isInterrupted();
    }

}
