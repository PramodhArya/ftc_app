package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name="GyroTest", group="Pushbot")
public class GyroTest extends LinearOpMode {

    BNO055IMU imu;

    Orientation angles;

    ElapsedTime runtime = new ElapsedTime();

    private DcMotor front = null;
    private DcMotor right = null;
    private DcMotor bottom = null;
    private DcMotor left = null;

    Float zeroAngle;
    float currentAngle;

    @Override
    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        front = hardwareMap.get(DcMotor.class, "front");
        right = hardwareMap.get(DcMotor.class, "right");
        bottom = hardwareMap.get(DcMotor.class, "bottom");
        left = hardwareMap.get(DcMotor.class, "left");

        front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front.setDirection(REVERSE);
        left.setDirection(REVERSE);

        while (!isStarted()) {
            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (zeroAngle == null)
                if (angles != null) {
                    zeroAngle = angles.firstAngle;
                    telemetry.addLine("zero angle intialized");
                    telemetry.addData("zero angle", zeroAngle);
                    telemetry.addData("angles firstAngle", angles.firstAngle);
                    telemetry.update();
                }
        }

        waitForStart();

        front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("starting turn");
        telemetry.update();

//        turnAbsolute(270);

        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("orientation", getCurrentAngle(angles));
            telemetry.addData("front position", front.getCurrentPosition());
            telemetry.addData("right position", right.getCurrentPosition());
            telemetry.addData("bottom position", bottom.getCurrentPosition());
            telemetry.addData("left position", left.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addLine("ending turn");
        telemetry.update();

    }

    public float getCurrentAngle(Orientation angles) {

        return (zeroAngle - (angles.firstAngle + 180) + 720) % 360;

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

    public void turnAbsolute(float angle) {

        float margin = 10;

        while (angles == null && opModeIsActive()) {}

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        float startingAngle = getCurrentAngle(angles);

        float leftMargin = (angle - margin + 360) % 360;
        float rightMargin = (angle + margin + 360) % 360;

        Range rangeOne;
        Range rangeTwo;

        if (leftMargin > rightMargin) {

            rangeOne = new Range(leftMargin, 360);
            rangeTwo = new Range(0, rightMargin);

        } else {

            rangeOne = new Range(leftMargin, rightMargin);
            rangeTwo = new Range(leftMargin, rightMargin);

        }

        while (!(rangeOne.contains(currentAngle) || rangeTwo.contains(currentAngle)) && opModeIsActive()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentAngle = getCurrentAngle(angles);

            telemetry.addData("current angle: ", currentAngle);
            telemetry.addData("left margin: ", leftMargin);
            telemetry.addData("right margin: ", rightMargin);
            telemetry.addData("within", (rangeOne.contains(currentAngle) || rangeTwo.contains(currentAngle)));
            telemetry.update();

        }

    }

}