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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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

@TeleOp(name="GyroTurnTest", group="Iterative Opmode")
//@Disabled
public class GyroTurnTest extends OpMode
{
    public Timer Diego;
    public TimerTask task;

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
    public void init() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
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

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

        front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (zeroAngle == null)
            if (angles != null)
                zeroAngle = angles.firstAngle;

        if (angles != null) {
//            telemetry.addData("orientation", getCurrentAngle(angles));
//            telemetry.addData("front position", front.getCurrentPosition());
//            telemetry.addData("right position", right.getCurrentPosition());
//            telemetry.addData("bottom position", bottom.getCurrentPosition());
//            telemetry.addData("left position", left.getCurrentPosition());
//            telemetry.update();
            turnAbsolute(270);
        }

    }

    @Override
    public void stop() {

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

        while (angles == null) {}

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

        while (!(rangeOne.contains(currentAngle) || rangeTwo.contains(currentAngle))) {

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
