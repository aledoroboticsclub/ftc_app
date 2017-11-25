package org.firstinspires.ftc.teamcode.Code9161_2017;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Aledo HS Robotics on 4/12/2017.
 */
//What is up my dudes.
public class Strider {

	DcMotor frontLeft;
	DcMotor frontRight;
	DcMotor backLeft;
	DcMotor backRight;
	Servo input1;
	Servo input2;
	Servo input3;
	Servo input4;
	/*ColorSensor revColor;
	ColorSensor MRColor;*/

	private ElapsedTime runtime = new ElapsedTime();

	public void initRobot(HardwareMap spareMap, Telemetry tempTelemetry) {
		Telemetry telemetry = tempTelemetry;
		frontLeft = spareMap.dcMotor.get("front left wheel");
		frontRight = spareMap.dcMotor.get("front right wheel");
		backLeft = spareMap.dcMotor.get("back left wheel");
		backRight = spareMap.dcMotor.get("back right wheel");
		frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		input1=spareMap.get(Servo.class, "input1");
		input2=spareMap.get(Servo.class, "input2");
		input3=spareMap.servo.get("output1");
		input4=spareMap.servo.get("output2");
		/*revColor=spareMap.colorSensor.get("REV Color Sensor");
		MRColor=spareMap.colorSensor.get("MR Color Sensor");*/
	}

	public void setToForward(double driveSpeed)
	{
		frontLeft.setPower(driveSpeed);
		frontRight.setPower(driveSpeed);
		backLeft.setPower(driveSpeed);
		backRight.setPower(driveSpeed);
	}

	public void setToBackward(double driveSpeed)
	{
		frontLeft.setPower(-1 * driveSpeed);
		frontRight.setPower(-1 * driveSpeed);
		backLeft.setPower(-1 * driveSpeed);
		backRight.setPower(-1 * driveSpeed);
	}

	public void setToClockwise(double driveSpeed)
	{
		frontLeft.setPower(-1 * driveSpeed);
		frontRight.setPower(1 * driveSpeed);
		backLeft.setPower(-1 * driveSpeed);
		backRight.setPower(1 * driveSpeed);
	}

	public void setToCounterwise(double driveSpeed)
	{
		frontLeft.setPower(1 * driveSpeed);
		frontRight.setPower(-1 * driveSpeed);
		backLeft.setPower(1 * driveSpeed);
		backRight.setPower(-1 * driveSpeed);
	}

	public void setToStill()
	{
		frontLeft.setPower(0);
		frontRight.setPower(0);
		backLeft.setPower(0);
		backRight.setPower(0);
	}

	public void setToLeft(double driveSpeed)
	{
		frontLeft.setPower(1 * driveSpeed);
		frontRight.setPower(-1 * driveSpeed);
		backLeft.setPower(-1 * driveSpeed);
		backRight.setPower(1 * driveSpeed);
	}

	public void setToRight(double driveSpeed)
	{
		frontLeft.setPower(-1 * driveSpeed);
		frontRight.setPower(1 * driveSpeed);
		backLeft.setPower(1 * driveSpeed);
		backRight.setPower(-1 * driveSpeed);
	}

	public void startIntake ()
	{
		input1.setPosition(0);
		input2.setPosition(1);
		input3.setPosition(0);
		input4.setPosition(1);
	}

	public void reverseIntake()
	{
		input1.setPosition(1);
		input2.setPosition(0);
		input3.setPosition(1);
		input4.setPosition(0);
	}

	public void stopIntake ()
	{
		input1.setPosition(.5);
		input2.setPosition(.5);
		input3.setPosition(.5);
		input4.setPosition(.5);
	}
}