package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.text.DecimalFormat;


@TeleOp(name="Main OpMode (BB)", group="BareBones")
public class BB_OpmodeIterative extends OpMode {
	// Configurations

	//  Declare members.
	//private final ElapsedTime runtime = new ElapsedTime();
	final public DcMotor[] motors = new DcMotor[4];
	final double[] wheelSpeeds = new double[4];
	//double fb,lr,turn,extraAxis;


	@Override
	public void init() {
		motors[0] = hardwareMap.get(DcMotor.class, "frontLeft");
		motors[1] = hardwareMap.get(DcMotor.class, "frontRight");
		motors[2] = hardwareMap.get(DcMotor.class, "backLeft");
		motors[3] = hardwareMap.get(DcMotor.class, "backRight");
	}

	@Override
	public void init_loop() {}

	@Override
	public void start() {
		//runtime.reset();
	}

	@Override
	public void loop() {
		//fb = -gamepad1.left_stick_y;
		//lr = gamepad1.left_stick_x;
		//turn  = gamepad1.right_stick_x;
		//extraAxis = -gamepad1.right_stick_y;

		mecanumDrive_Cartesian(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
	}

	public void mecanumDrive_Cartesian(double y,double x,double rotation) {
		wheelSpeeds[0] = y+x+rotation;
		wheelSpeeds[1] = y-x-rotation;
		wheelSpeeds[2] = y-x+rotation;
		wheelSpeeds[3] = y+x-rotation;

		double top = Math.max(Math.abs(wheelSpeeds[0]),
				Math.max(Math.abs(wheelSpeeds[1]),
						Math.max(Math.abs(wheelSpeeds[2]), Math.abs(wheelSpeeds[3]))));

		// 1. Divides wheelSpeeds proportionally
		for (int i = 0; i < wheelSpeeds.length; i++)
			if (wheelSpeeds[i] > 1.0 || wheelSpeeds[i] < -1.0)
				wheelSpeeds[i] = Double.parseDouble(new DecimalFormat("0.000").format(wheelSpeeds[i] / top));

		// Send calculated power to motors
		motors[0].setPower(-wheelSpeeds[0]);
		motors[1].setPower(wheelSpeeds[1]);
		motors[2].setPower(-wheelSpeeds[2]);
		motors[3].setPower(wheelSpeeds[3]);
	}

	@Override
	public void stop() {
		// Slowly turn off the motors after pressing stop
		motors[0].setPower(0.0);
		motors[1].setPower(0.0);
		motors[2].setPower(0.0);
		motors[3].setPower(0.0);
	}
}