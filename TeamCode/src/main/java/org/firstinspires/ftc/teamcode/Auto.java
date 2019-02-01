package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@Autonomous(name="Autonomous Depot", group="Pushbot")
public class Auto extends LinearOpMode {

    /* Declare OpMode members. */
//    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device

    static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 1;     // Nominal speed for better accuracy.
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
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

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

        bottom.setDirection(REVERSE);
        right.setDirection(REVERSE);

        colorCheck();

        waitForStart();

        unhook(24500);
//        moveX(DRIVE_SPEED, 200);
//        gyroTurn(DRIVE_SPEED, -20);
//        pause();
//        moveY(DRIVE_SPEED,1000);
//        pause();
//        gyroTurn(TURN_SPEED, -4);
//        pause();
    }

    public void moveY(double power, int distance) {
        front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front.setTargetPosition(distance);
        bottom.setTargetPosition(distance);
        front.setPower(power);
        bottom.setPower(power);
        while (opModeIsActive() && front.isBusy() && bottom.isBusy()) {
            telemetry.addData("Encoder", front.getCurrentPosition());
            telemetry.update();
            idle();
        }
        front.setPower(0);
        bottom.setPower(0);

        front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveX(double power, int distance) {
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setTargetPosition(distance);
        left.setTargetPosition(distance);
        right.setPower(power);
        left.setPower(power);
        while (opModeIsActive() && right.isBusy() && left.isBusy()) {
            telemetry.addData("Right", right.getCurrentPosition());
            telemetry.addData("Left", left.getCurrentPosition());
            telemetry.update();
            idle();
        }
        right.setPower(0);
        left.setPower(0);

        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    }

    public void sampleMineral() {
        switch (goldMineralPosition)
        {
            case LEFT:
                telemetry.addData("Gold mineral is on the","left");
                break;
            case MIDDLE:
                telemetry.addData("Gold mineral is on the","middle");
                break;
            case RIGHT:
                telemetry.addData("Gold mineral is on the","right");
                break;
        }
    }

    public void unhook(int value) {
        lifter.setTargetPosition(value);
        lifter.setPower(1);
        while (opModeIsActive() && lifter.isBusy()) {
            telemetry.addData("Lifter", lifter.getCurrentPosition());
            telemetry.update();
            idle();
        }
        lifter.setPower(0);
    }

}