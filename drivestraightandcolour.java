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

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This Opmode is an example of a Proportional Controller
 * The drivetrain will move to a target distance using encoders and slow down when it approaches using proportional
 *
 * ---------------------------- PROPORTIONAL ------------------------------------
 * Error = Target - Current Position
 * Target is the distance that the robot will move to
 * Current position is an average of all the encoders
 * Kp is a constant that needs to be tuned
 * Gain = Error*Kp
 * velocity = speed*gain
 * velocity will store the result
 * speed is a constant chosen by the user
 * gain will decrease as the drivetrain approaches the target
 *
 * Proportional controller is a good solution for controlling a drivetrain autonomously,
 * but the robot will never reach the target, there will always be a steady state error.
 * The Steady State Error will not be too big, but it can make the code get stuck in the PDrive loop.
 * The solution for that is to exit the loop once the error is very small.
 *
 * ---------------------------- FTC DASHBOARD ------------------------------------
 * https://acmerobotics.github.io/ftc-dashboard/
 * Kp and target distance can be changed using the dashboard
 * Prints a graph of the position
 * To open the dashboard connect your laptop to the robot's wifi and then access this address using a browser:
 * http://192.168.43.1:8080/dash
 */

@Autonomous(name="tieriii", group="Linear Opmode")
//@Disabled
public class drivestraightandcolour extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    Orientation angles;

    private ColorSensor colorSensor;
    //the RGB thingy number
    final double SCALE_FACTOR = 255;

    Servo servo;

    // Declare OpMode members.
    private DcMotor leftDrive = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive = null;
    private DcMotor rightDrive2 = null;

    public static  double     P_DRIVE_COEFF = 0.00202674492; // kp=1/(704.86*constant) = constant = 0.7; //0.04

    static final double     COUNTS_PER_MOTOR_REV    = 704.86 ;    //
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        servo = hardwareMap.get(Servo.class, "left_hand");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        leftDrive2 = hardwareMap.get(DcMotor.class, "left_drive2");
        rightDrive  = hardwareMap.get(DcMotor.class, "right_drive");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right_drive2");

        // Most robots need the motor on one side to be reversed to drive forward
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        gyroDrive(0.1,30,0);

    }
    public void printColor(){

        float hsvValues[] = {0f, 0f, 0f};

        /*convert the RGB values to HSV values. int converts values to integers
            things thing only accepts integers, so we convert the floats to integers*/
        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR),
                hsvValues);

            /*the hsvValues[] retrieves the value of the hue, saturation, and brightness on the color sensor
            see the function below. it reports up to here.*/
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Saturation", hsvValues[1]);
        telemetry.addData("Value", hsvValues[2]);

        //updates when color is detected
            /*e.g. if the color is red, as in according to the parameters set by the function below
            it will print red*/
        if (colorTest("red", hsvValues)) {
            telemetry.addData("found red", 1);
            servo.setPosition(1);
        } else if (colorTest("blue", hsvValues)) {
            telemetry.addData("found blue", 1);
            servo.setPosition(0.5);
        } else if (colorTest("orange", hsvValues)) {
            telemetry.addData("found orange", 1);
            servo.setPosition(0);
        } else {
            telemetry.addData("nothing", 1);
        }
        //the 1 is just there to fill space
        telemetry.update();
        //resets runtime
        runtime.reset();
    }
    public void gyroDrive(double speed, double distance, double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = rightDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftDrive.setTargetPosition(newLeftTarget);
            leftDrive2.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);
            rightDrive2.setTargetPosition(newRightTarget);

            //runs to target position
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            speed = Range.clip(Math.abs(speed), 0.0, 1.0);


            // making sure all motors are running
            while (opModeIsActive() &&
                    leftDrive.isBusy() && rightDrive.isBusy() && leftDrive2.isBusy() && rightDrive2.isBusy()) {

                // adjust relative speed based on heading error.
                error = getErrorAngle(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                    //divides the speeds by themselves so that the max = 1 always
                }

                printColor();

                telemetry.update();

                leftDrive.setPower(leftSpeed);
                leftDrive2.setPower(leftSpeed);

                rightDrive.setPower(rightSpeed);
                rightDrive2.setPower(rightSpeed);
            }

            // Stop all motion;
            //stopMotors();

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public double getCurrentPosition(){
        return Math.abs(leftDrive.getCurrentPosition() + leftDrive2.getCurrentPosition()
                + rightDrive.getCurrentPosition() + rightDrive2.getCurrentPosition())/4;
        // Math.abs returns the absolute value of an argument
    }
    public double getErrorAngle(double targetAngle) {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    public void resetEncoders(){
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void stopMotors(){
        leftDrive.setPower(0);
        leftDrive2.setPower(0);
        rightDrive.setPower(0);
        rightDrive2.setPower(0);
    }
    public void print(double target, Telemetry dashboardTelemetry){

        dashboardTelemetry.addData("Angle", angles.firstAngle);
        dashboardTelemetry.addData("Distance", getCurrentPosition()/COUNTS_PER_INCH);
        dashboardTelemetry.update();
    }
    public boolean colorTest(String color, float[] array) {

        //the array[] thingy thing is just whatever number is detected by the sensor for color
        //which then gets popped back up in the printing and whatnot
        //also all numbers start from 0
        float hue = array[0];
        float sat = array[1];
        float val = array[2];

        //sets the parameters for what is red, wat is blue, wat is yellow
        if (color.equals("red")) {
            if (35 < hue && hue < 70 && 0.2 < sat && sat < 0.5 && 75 < val && val < 140) {
                return true;
            } else {
                return false;
            }
        } else if (color.equals("blue")) {
            if (140 < hue && hue < 250 && 0.4 < sat && sat < 0.9 && 70 < val && val < 120) {
                return true;
            } else {
                return false;
            }
        } else if (color.equals("orange")) {
            if (70 < hue && hue < 120 && 0.3 < sat && sat < 0.9 && 80 < val && val < 300) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }
}