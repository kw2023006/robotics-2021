package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="maglimsens", group="Linear Opmode")
//@Disabled

public class magneticlimitswitch extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;

    //declares the sensor
    DigitalChannel magneticSensor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        leftDrive2 = hardwareMap.get(DcMotor.class, "left_drive_2");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right_drive_2");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);

        //sets it up in the system within the robot
        magneticSensor = hardwareMap.get(DigitalChannel.class, "magnetic_sensor");
        magneticSensor.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();
        runtime.reset();
        //the sensor is false when something is pressing on it and true when it has freedom
        //! means false
        while (opModeIsActive()) {

            if (!magneticSensor.getState()) {
                stopMotors();
                telemetry.addData("Digital Touch", "False");
            } else if (magneticSensor.getState()){
                moveMotors();
                telemetry.addData("Digital Touch", "True");
            }
            telemetry.update();
        }
    }
    public void stopMotors(){
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive2.setPower(0);
        rightDrive2.setPower(0);
    }
    public void moveMotors(){
        leftDrive.setPower(0.3);
        rightDrive.setPower(0.3);
        leftDrive2.setPower(0.3);
        rightDrive2.setPower(0.3);
    }
}