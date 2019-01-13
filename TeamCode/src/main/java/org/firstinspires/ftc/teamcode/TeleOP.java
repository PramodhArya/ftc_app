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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOP", group="Iterative Opmode")
public class TeleOP extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor topRight = null;
    private DcMotor bottomRight = null;
    private DcMotor bottomLeft = null;
    private DcMotor topLeft = null;
    private DcMotor intake = null;

    private double powerScale = 1;

    private double leftY1;
    private double leftX1;
    private double rightX1;


    @Override
    public void init() {
        //Hardware Maps
        topRight = hardwareMap.get(DcMotor.class, "topRight");
        bottomRight = hardwareMap.get(DcMotor.class, "bottomRight");
        bottomLeft = hardwareMap.get(DcMotor.class, "bottomLeft");
        topLeft = hardwareMap.get(DcMotor.class, "topLeft");
//        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        leftY1 = powerCurve(gamepad1.left_stick_y);
        leftX1 = powerCurve(gamepad1.left_stick_x);
        rightX1 = powerCurve(gamepad1.right_stick_x);

        //Movement
        if (gamepad1.left_stick_y > 0) {
            fwd();
        } else if (gamepad1.left_stick_y < 0) {
            bwd();
        } else {
            still();
        }

        if (gamepad1.left_stick_x > 0) {
            right();
        } else if (gamepad1.left_stick_x < 0) {
            left();
        } else {
            still();
        }

        //Rotation
        if (gamepad1.right_stick_x > 0) {
            turnRight();
        } else if (gamepad1.right_stick_x < 0) {
            turnLeft();
        } else {
            still();
        }

//        //Intake
//        if (gamepad1.right_trigger != 0) {
//            intake.setPower(gamepad1.right_trigger);
//        } else if (gamepad1.left_trigger != 0) {
//            intake.setPower(gamepad1.left_trigger );
//        } else {
//            intake.setPower(0);
//        }
    }

    @Override
    public void stop() {
    }

    public void fwd() {
        topRight.setPower(-leftY1);
        bottomRight.setPower(-leftY1);
        bottomLeft.setPower(leftY1);
        topLeft.setPower(leftY1);
    }

    public void bwd() {
        topRight.setPower(leftY1);
        bottomRight.setPower(leftY1);
        bottomLeft.setPower(-leftY1);
        topLeft.setPower(-leftY1);
    }

    public void right() {
        topRight.setPower(-leftX1);
        bottomRight.setPower(leftX1);
        bottomLeft.setPower(leftX1);
        topLeft.setPower(-leftX1);
    }

    public void left() {
        topRight.setPower(leftX1);
        bottomRight.setPower(-leftX1);
        bottomLeft.setPower(-leftX1);
        topLeft.setPower(leftX1);
    }

    public void turnRight() {
        topRight.setPower(-rightX1);
        bottomRight.setPower(-rightX1);
        bottomLeft.setPower(-rightX1);
        topLeft.setPower(-rightX1);
    }

    public void turnLeft() {
        topRight.setPower(rightX1);
        bottomRight.setPower(rightX1);
        bottomLeft.setPower(rightX1);
        topLeft.setPower(rightX1);
    }

    public void still() {
        topRight.setPower(0);
        bottomRight.setPower(0);
        bottomLeft.setPower(0);
        topLeft.setPower(0);
    }

    public double powerCurve(double power) {
        return powerScale*Math.pow(power,2);
    }




}
