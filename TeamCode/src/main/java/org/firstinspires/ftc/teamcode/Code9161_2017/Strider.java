package org.firstinspires.ftc.teamcode.Code9161_2017;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Aledo HS Robotics on 4/12/2017.
 */
//What is up my dudes.
public class Strider
{

	DcMotor frontLeft;
	DcMotor frontRight;
	DcMotor backLeft;
	DcMotor backRight;
	Servo spinner1;
	Servo spinner2;

	private ElapsedTime runtime=new ElapsedTime();

	public void initRobot(HardwareMap spareMap, Telemetry tempTelemetry)
	{
		Telemetry telemetry=tempTelemetry;
		frontLeft=spareMap.dcMotor.get("front left wheel");
		frontRight=spareMap.dcMotor.get("front right wheel");
		backLeft=spareMap.dcMotor.get("back left wheel");
		backRight=spareMap.dcMotor.get("back right wheel");
		frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		spinner1=spareMap.get(Servo.class, "spinner1");
		spinner2=spareMap.get(Servo.class, "spinner2");
	}

	public void setToForward ()
	{
		frontLeft.setPower(1);
		frontRight.setPower(1);
		backLeft.setPower(1);
		backRight.setPower(1);
	}

	public void setToBackward ()
	{
		frontLeft.setPower(-1);
		frontRight.setPower(-1);
		backLeft.setPower(-1);
		backRight.setPower(-1);
	}

	public void setToClockwise ()
	{
		frontLeft.setPower(1);
		frontRight.setPower(-1);
		backLeft.setPower(1);
		backRight.setPower(-1);
	}

	public void setToCounterclockwise ()
	{
		frontLeft.setPower(-1);
		frontRight.setPower(1);
		backLeft.setPower(-1);
		backRight.setPower(1);
	}

	public void setToStill ()
	{
		frontLeft.setPower(0);
		frontRight.setPower(0);
		backLeft.setPower(0);
		backRight.setPower(0);
	}

	public void setToLeft ()
	{
		frontLeft.setPower(-1);
		frontRight.setPower(1);
		backLeft.setPower(1);
		backRight.setPower(-1);
	}

	public void setToRight ()
	{
		frontLeft.setPower(1);
		frontRight.setPower(-1);
		backLeft.setPower(-1);
		backRight.setPower(1);
	}

	public void runIntake ()
	{
		spinner1.setPosition(0);
		spinner2.setPosition(1);
	}

	public void stopIntake ()
	{
		spinner1.setPosition(.5);
		spinner2.setPosition(.5);
	}
}