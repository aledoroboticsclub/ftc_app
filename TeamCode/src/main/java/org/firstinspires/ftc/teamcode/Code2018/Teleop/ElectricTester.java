package org.firstinspires.ftc.teamcode.Code2018.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Code2018.Scorpion;

@TeleOp(name="ElectricTester", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled
     //Determines if the program shows up on Driver Station


public class ElectricTester extends OpMode
{
	Scorpion r=new Scorpion();
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

	public void loop() {
		r.frontLeft.setPower(gamepad1.left_stick_y);
		r.frontRight.setPower(gamepad1.right_stick_y);
		r.leftIntake(gamepad1.left_trigger);
		r.rightIntake(gamepad1.right_trigger);
		if(gamepad1.b)
			r.extenderOut();
		else if(gamepad1.x)
			r.extenderIn();
		else
			r.extenderStill();
	}
}
