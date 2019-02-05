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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

@TeleOp(name="TeleOP", group="Iterative Opmode")
public class TeleOP extends OpMode
{

    double COUNTS_PER_REVOLUTION = 1120;
    double GEAR_REDUCTION = 3;
    double PULLEY_DIAMETER = 1.456;
    double HEIGHT_OF_AXIS = 14;
    double angleIncrement = Math.PI/36;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front = null;
    private DcMotor right = null;
    private DcMotor bottom = null;
    private DcMotor left = null;
    private DcMotor lifter = null;
    private DcMotor flipper = null;

    private Servo winch = null;
    private Servo tilt = null;
    private CRServo intake = null;
    
    private double leftY1;
    private double leftX1;
    private double rightX1;
    private double leftTrigger2;
    private double rightTrigger2;
    private int counter = 0;
    private int targetCount = 70;

    private double winchIncrement = .05;
    private double minWinch = 0.25;
    private double maxWinch = 1.0;

    private double up = 135.0/180.0;
    private double down = 0.0/180.0;
    private double up1 = 155.0/180.0;
    private double middle = 80.0/180.0;

//    double COUNTS_PER_REVOLUTION = 1120; //for 40, 1680 for 60
//    double GEAR_REDUCTION = 27;
//    double PULLEY_DIAMETER = 1.456;
//    double HEIGHT_OF_AXIS = 1;//TODO: UNKNOWN

    boolean pressed = false;

    @Override
    public void init() {
        //Hardware Maps
        front = hardwareMap.get(DcMotor.class, "front");
        right = hardwareMap.get(DcMotor.class, "right");
        bottom = hardwareMap.get(DcMotor.class, "bottom");
        left = hardwareMap.get(DcMotor.class, "left");
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        flipper = hardwareMap.get(DcMotor.class, "flipper");

        winch = hardwareMap.get(Servo.class, "winch");
        tilt = hardwareMap.get(Servo.class, "tilt");
        intake = hardwareMap.get(CRServo.class, "intake");

//        flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        flipper.setZeroPowerBehavior(BRAKE);

    }
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
//        flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        flipper.setPower(.3);
        runtime.reset();
        winch.setPosition(minWinch);
    }

    @Override
    public void loop() {


        leftY1 = powerScale(gamepad1.left_stick_y,.7);
        leftX1 = powerScale(gamepad1.left_stick_x,.7);
        rightX1 = powerScale(gamepad1.right_stick_x,.7);

        leftTrigger2 = powerScale(gamepad2.left_trigger, .5);
        rightTrigger2 = powerScale(gamepad2.right_trigger,.7);

        //Movement
        front.setPower(-leftX1 - rightX1);
        right.setPower(-leftY1 - rightX1);
        bottom.setPower(leftX1 - rightX1);
        left.setPower(leftY1 - rightX1);

        //Flipper
        if (gamepad2.right_trigger != 0) {
            flipper.setPower(-rightTrigger2);
        } else if (gamepad2.left_trigger != 0) {
            flipper.setPower(leftTrigger2);
        } else {
            flipper.setPower(0);
        }
//        if (gamepad2.right_trigger != 0) {
//            flipper.setTargetPosition(flipper.getCurrentPosition() + RadiansToCounts(angleIncrement));
//        } else if (gamepad2.left_trigger != 0) {
//            flipper.setTargetPosition(flipper.getCurrentPosition() - RadiansToCounts(angleIncrement));
//        }

        //Intake
        if (gamepad1.right_trigger != 0) {
            intake.setPower(-gamepad1.right_trigger);
        } else if (gamepad1.left_trigger != 0) {
            intake.setPower(gamepad1.left_trigger);
        } else {
            intake.setPower(0);
        }
        //Box Tilt
        if (gamepad1.a) {
            tilt.setPosition(up);
        } else if (gamepad1.b) {
            tilt.setPosition(down);
        } else if (gamepad1.x) {
            tilt.setPosition(up1);
        } else if (gamepad1.y) {
            tilt.setPosition(middle);
        }

        //Linear Actuator
        if (gamepad2.dpad_up) {
            lifter.setPower(1);
        } else if (gamepad2.dpad_down) {
            lifter.setPower(-1);
        } else {
            lifter.setPower(0);
        }

        //Winch
        if (!pressed) {
            if (gamepad2.right_bumper && (winch.getPosition() + winchIncrement < maxWinch)) {
                winch.setPosition(winch.getPosition() + winchIncrement);
                pressed = true;
            } else if (gamepad2.left_bumper && (winch.getPosition() - winchIncrement > minWinch)) {
                winch.setPosition(winch.getPosition() - winchIncrement);
                pressed = true;
            }
        } else if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
            pressed = false;
        }

/*        if (counter < targetCount) counter++;
        if (gamepad2.right_bumper && counter == targetCount) {
            winch.setPosition(winch.getPosition() + winchIncrement);
            counter = 0;
        } else if (gamepad2.left_bumper && counter == targetCount) {
            winch.setPosition(winch.getPosition() - winchIncrement);
            counter = 0;
        } else {
            winch.setPosition(winch.getPosition());
        }*/

        telemetry.addData("Winch Position: ", winch.getPosition());
        telemetry.update();

    }

    @Override
    public void stop() {
    }

    public double powerScale(double power, double powerScaleFactor) {
        if (power > 0) {
            return Math.pow(power,2)*powerScaleFactor;
        } else if (power < 0) {
            return -Math.pow(power,2)*powerScaleFactor;
        } else {
            return 0;
        }
    }

//    public void PolarExtension(double theta, double radius) {
//
//        winch.setPosition(radius / (Math.PI * PULLEY_DIAMETER));
//        flipper.setTargetPosition();
//    }

//    public void FloorExtension(double x) {
//
//        double radius = Math.sqrt(x * x + HEIGHT_OF_AXIS * HEIGHT_OF_AXIS);
//        double theta = Math.asin(x / radius);
//
//        PolarExtension(theta, radius);
//
//    }

    public double CountsToRadians(int counts) {

        return (counts * 2 * Math.PI) / (COUNTS_PER_REVOLUTION * GEAR_REDUCTION);

    }

    public int RadiansToCounts(double radians) {

        return (int)((radians * COUNTS_PER_REVOLUTION * GEAR_REDUCTION) / (2 * Math.PI));

    }
}
