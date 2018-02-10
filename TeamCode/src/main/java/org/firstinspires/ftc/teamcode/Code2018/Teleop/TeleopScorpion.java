package org.firstinspires.ftc.teamcode.Code2018.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Code2018.Scorpion;

@TeleOp(name="TeleopScorpion", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
   //Determines if the program shows up on Driver Station


public class TeleopScorpion extends OpMode
{
	Scorpion r=new Scorpion();
	ElapsedTime runtime=new ElapsedTime();
	public void init() {
		telemetry.addLine("init begun");
		r.initRobot(hardwareMap,telemetry);
		r.initGyro();
		telemetry.addLine("init Complete");

		telemetry.update();
	}

	public void initLoop(){}

	public void start(){
		runtime.reset();
	}

	boolean inReverse=true;//reverse button is a
	boolean aWasPressed=false;
	public void loop() {
		if(gamepad1.a && !aWasPressed)
			inReverse=!inReverse;
		aWasPressed=gamepad1.a;
		//first we must translate the rectangular values of the joystick into polar coordinates
		double y=-1*gamepad1.left_stick_y;
		double x=-1*gamepad1.left_stick_x;
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
		angle+=Math.toRadians(90);
		//now we get the velocity of the robot based on the distance the disanctance the sticks are from the origen
		double velocity=Math.sqrt(Math.pow(gamepad1.left_stick_y, 2)+Math.pow(gamepad1.left_stick_x, 2));
		//now we get the rotation simply based on the right stick's horizontal x value
		double rotation=gamepad1.right_stick_x*-1;

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


		//tray
		if(gamepad2.a)
			r.setTrayToIntake();
		else if(gamepad2.y)
			r.setTrayToPlace();
		else if(gamepad2.x)
			r.setTrayToParallel();

		//intake
		if(gamepad2.right_trigger>.1)
			r.rightIntake(gamepad2.right_trigger);
		else if(gamepad2.right_bumper)
			r.rightIntake(-1);
		else
			r.rightIntake(0);
		if (gamepad2.left_trigger>.1)
			r.leftIntake(gamepad2.left_trigger);
		else if(gamepad2.left_bumper)
			r.leftIntake(-1);
		else
			r.leftIntake(0);

		//lift
		if(gamepad2.left_stick_y>.1)
			r.moveLiftUp(gamepad2.left_stick_y);
		else if(gamepad2.left_stick_y<-.1)
			r.moveLiftDown(gamepad2.left_stick_y*-1);

		//lift
		if (gamepad2.dpad_down)
			r.setLiftToPosition0();//placing over ground and loading
		else if (gamepad2.dpad_left)
			r.setLiftToPosition1();//placing over 1 cube
		else if (gamepad2.dpad_right)
			r.setLiftToPosition2();//placing over 2 cubes
		else if (gamepad2.dpad_up)
			r.setLiftToPosition3();//placing over 3 cubes

		//relic
		if (gamepad2.right_stick_x<-.75)
			r.setGrabberToGrabbed();
		else if (gamepad2.right_stick_x>.75)
			r.setGrabberToRelease();

		//extender
		if(gamepad2.right_stick_y>.25)
			r.extenderOut();
		else if(gamepad2.right_stick_y<-.25)
			r.extenderIn();
		else
			r.extenderStill();
		telemetry.addData("Runtime: ", runtime.seconds());
		telemetry.addData("RevHub1 Angle", r.getGyroAvgZ1());
		telemetry.addData("RevHub2 Angle", r.getGyroAvgZ2());
		telemetry.update();
	}
}
