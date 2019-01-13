//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//
//
//@Autonomous(name="Autonomous Depot", group="Pushbot")
//public class AutonomousDepot extends LinearOpMode {
//
//    /* Declare OpMode members. */
////    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device
//
//    static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: TETRIX Motor Encoder
//    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
//    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_DIAMETER_INCHES * 3.1415);
//
//    // These constants define the desired driving/control characteristics
//    // The can/should be tweaked to suite the specific robot drive train.
//    static final double     DRIVE_SPEED             = 1;     // Nominal speed for better accuracy.
//    static final double     TURN_SPEED              = 0.7;     // Nominal half speed for better accuracy.
//
//    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
//    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
//    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
//
//    static final double SERVO_DOWN = 30/280.0;
//    static final double SERVO_UP = 180/280.0;
//
//    boolean gold = false;
//
//    ElapsedTime timer = new ElapsedTime();
//
//    BNO055IMU imu;
//
//    Orientation angles;
//    Acceleration gravity;
//
//    @Override
//    public void runOpMode() {
//
//        //*************************** IMU Setup ************************/
//
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
//
//        //*************************** IMU Setup ************************/
//
//        //TODO: HARDWARE MAPS
//
//        waitForStart();
//        //TODO: ACTUAL CODE LOL
//
//    }
//
//    public void gyroDrive ( double speed,
//                            double distance,
//                            double angle) {
//
//        int     newLeftTarget;
//        int     newRightTarget;
//        int     moveCounts;
//        double  max;
//        double  error;
//        double  steer;
//        double  leftSpeed;
//        double  rightSpeed;
//
//        if (opModeIsActive()) {
//
//            moveCounts = (int)(distance * COUNTS_PER_INCH);
//            newLeftTarget = motorLeft.getCurrentPosition() + moveCounts;
//            newRightTarget = motorRight.getCurrentPosition() + moveCounts;
//
//            motorLeft.setTargetPosition(newLeftTarget);
//            motorRight.setTargetPosition(newRightTarget);
//
//            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
//            motorLeft.setPower(speed);
//            motorRight.setPower(speed);
//
//            while (opModeIsActive() &&
//                    (motorLeft.isBusy() && motorRight.isBusy())) {
//
//                error = getError(angle);
//                steer = getSteer(error, P_DRIVE_COEFF);
//
//                if (distance < 0)
//                    steer *= -1.0;
//
//                leftSpeed = speed - steer;
//                rightSpeed = speed + steer;
//
//                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
//                if (max > 1.0)
//                {
//                    leftSpeed /= max;
//                    rightSpeed /= max;
//                }
//
//                motorLeft.setPower(leftSpeed);
//                motorRight.setPower(rightSpeed);
//
//                telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//                telemetry.addData("LEFT POSITION", motorLeft.getCurrentPosition());
//                telemetry.addData("RIGHT POSITION", motorRight.getCurrentPosition());
//                telemetry.update();
//            }
//
//            motorLeft.setPower(0);
//            motorRight.setPower(0);
//
//            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//    }
//
//    public void gyroTurn (  double speed, double angle) {
//
//        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
//            telemetry.addData("heading", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//            telemetry.update();
//        }
//        motorHDrive.setPower(0);
//    }
//
//    public void gyroHold( double speed, double angle, double holdTime) {
//
//        ElapsedTime holdTimer = new ElapsedTime();
//
//        holdTimer.reset();
//        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
//            onHeading(speed, angle, P_TURN_COEFF);
//            telemetry.update();
//        }
//
//        motorLeft.setPower(0);
//        motorRight.setPower(0);
//
//    }
//
//    boolean onHeading(double speed, double angle, double PCoeff) {
//        double   error ;
//        double   steer ;
//        boolean  onTarget = false ;
//        double leftSpeed;
//        double rightSpeed;
//
//        error = getError(angle);
//
//        if (Math.abs(error) <= HEADING_THRESHOLD) {
//            steer = 0.0;
//            leftSpeed  = 0.0;
//            rightSpeed = 0.0;
//            onTarget = true;
//        }
//        else {
//            steer = getSteer(error, PCoeff);
//            rightSpeed  = speed * steer;
//            leftSpeed   = -rightSpeed;
//        }
//
//        motorLeft.setPower(leftSpeed);
//        motorRight.setPower(rightSpeed);
//
//        telemetry.addData("Target", "%5.2f", angle);
//        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
//        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
//
//        return onTarget;
//    }
//
//    public double getError(double targetAngle) {
//
//        double robotError;
//
//        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//        while (robotError > 180)  robotError -= 360;
//        while (robotError <= -180) robotError += 360;
//        return robotError;
//    }
//
//    public double getSteer(double error, double PCoeff) {
//        return Range.clip(error * PCoeff, -1, 1);
//    }
//
//}