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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(name="test", group="Iterative Opmode")
@Disabled
public class Test extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intake = null;
    private Servo pully = null;
    private CRServo i = null;
    private Servo tilt = null;

    private boolean a = false;
    private int counter = 0;
    private int targetCount = 70;
    private double pullyIncrement = 0.05;
    private double up = 45.0/180.0;
    private double down = 0.0/180.0;

    private final int gearReduction = 27;
    private final double innerDiameter = 1.456;
    private final double axelHight = 15.375;
    private final double countsPerRevolution = 1120;

    public void polarExtension(double radius, double theta) {

//        pully.setPosition(radius/(innerDiameter*Math.PI*8);
        intake.setTargetPosition((int)(theta * 1120 / (360)));

    }

    @Override
    public void init() {
        //Hardware Maps
        intake = hardwareMap.get(DcMotor.class, "intake");
        pully = hardwareMap.get(Servo.class, "pully");
        i = hardwareMap.get(CRServo.class, "i");
        tilt = hardwareMap.get(Servo.class, "tilt");
        intake.setZeroPowerBehavior(BRAKE);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        //Intake
        if (gamepad1.right_trigger != 0) {
            intake.setPower(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger != 0) {
            intake.setPower(-gamepad1.left_trigger );
        } else {
            intake.setPower(0);
        }

        if (gamepad1.a) {
            i.setPower(1);
        } else if (gamepad1.b) {
            i.setPower(-1);
        } else {
            i.setPower(0);
        }
        if (gamepad1.x) {
            tilt.setPosition(up);
        } else if (gamepad1.y) {
            tilt.setPosition(down);
        }

        if(gamepad1.dpad_down) {
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else if(gamepad1.dpad_up) {
            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (counter < targetCount) counter++;
        if (gamepad1.right_bumper && counter == targetCount) {
            pully.setPosition(pully.getPosition() + pullyIncrement);
            counter = 0;
        } else if (gamepad1.left_bumper && counter == targetCount) {
            pully.setPosition(pully.getPosition() - pullyIncrement);
            counter = 0;
        } else {
            pully.setPosition(pully.getPosition());
        }
        telemetry.addData("Servo Position", pully.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
    }

}
