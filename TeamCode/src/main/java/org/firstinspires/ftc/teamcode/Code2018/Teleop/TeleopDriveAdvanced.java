package org.firstinspires.ftc.teamcode.Code2018.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Code2018.Scorpion;

import java.text.DecimalFormat;
//all motor values are independantly determined by formulas found on internet for mecanum drive
//is analog (joystick based), thus TeleopDriveStick is no longer needed
//is now being used as drive system in TeleopScorpion
//complete

@TeleOp(name="TeleopDriveAdvanced", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled



public class TeleopDriveAdvanced extends OpMode
{
	Scorpion r=new Scorpion();
	DecimalFormat df = new DecimalFormat("#.00");
	ElapsedTime runtime=new ElapsedTime();
	public void init() {
		r.initRobot(hardwareMap,telemetry);
		telemetry.addData("init Complete","");

		telemetry.update();
	}

	public void initLoop(){}

	public void start(){
		runtime.reset();
	}

	boolean inReverse=false;//reverse button is a
	boolean aWasPressed=false;
	public void loop() {
		if(gamepad1.a && !aWasPressed)
			inReverse=!inReverse;
		aWasPressed=gamepad1.a;
		//first we must translate the rectangular values of the joystick into polar coordinates;
		double y=-1*gamepad1.left_stick_y;
		double x=gamepad1.left_stick_x;
		double angle=0;
		if(y>0 && x>0)//quadrant 1
			angle=Math.atan(y/x);
		else if(y>0 && x<0)//quadrant 2
			angle=Math.toRadians(180)+Math.atan(y/x);
		else if(y<0 && x<0)//quadrant 3
			angle=Math.toRadians(180)+Math.atan(y/x);
		else if(y<0 && x>0)//quadrant 4
			angle=Math.toRadians(360)+Math.atan(y/x);
		if(y==0 && x>1)
			angle=0;
		if(y>0 && x==0)
			angle=Math.PI/2;
		if(y==0 && x<0)
			angle=Math.PI;
		if(y<0 && x==0)
			angle=3*Math.PI/2;
		double velocity=Math.sqrt(Math.pow(gamepad1.left_stick_y, 2)+Math.pow(gamepad1.left_stick_x, 2));
		double rotation=gamepad1.right_stick_x;

		if(inReverse)//reverse button
			angle+=Math.toRadians(180);

		//equations taking the polar coordinates and turing them into motor powers
		double power1=velocity*Math.cos(angle+(Math.PI/4))-rotation;
		double power2=velocity*Math.sin(angle+(Math.PI/4))+rotation;
		double power3=velocity*Math.sin(angle+(Math.PI/4))-rotation;
		double power4=velocity*Math.cos(angle+(Math.PI/4))+rotation;
		r.frontLeft.setPower(power1);
		r.frontRight.setPower(power2);
		r.backLeft.setPower(power3);
		r.backRight.setPower(power4);

		/*telemetry.addData("Degrees: ", Math.round(Math.toDegrees(angle)));
		telemetry.addData("Velocity: ", Range.clip(velocity, -1, 1));
		telemetry.addData("Rotation: ", rotation);
		telemetry.addData("Power1: ", power1);
		telemetry.addData("Power2: ", power2);
		telemetry.addData("Power3: ", power3);
		telemetry.addData("Power4: ", power4);*/

	}
}

