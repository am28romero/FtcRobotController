/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.imageproc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name="Object Tracking", group="Snippets")
public class ObjectTracking extends LinearOpMode {
	OpenCvCamera phoneCam;
	final Size cameraRes = new Size(320, 240);
	final int cameraType = 0;
	Pipeline pipeline = new Pipeline();

	@Override
	public void runOpMode() {
		//  Make an instance of the phone camera
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

		phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				// Start Streaming
				phoneCam.startStreaming((int) cameraRes.width, (int) cameraRes.height, OpenCvCameraRotation.UPRIGHT);
			}
		});


		telemetry.addLine("HSV Values:  ")
				.addData("Hue",pipeline.maxValues.val[0] + " - " + pipeline.minValues.val[0])
				.addData("Saturation",pipeline.maxValues.val[1] + " - " + pipeline.minValues.val[1])
				.addData("Value (Brightness)",pipeline.maxValues.val[2] + " - " + pipeline.minValues.val[2]);
		telemetry.addLine("Setup Finished Successfully. Waiting for Start...");
		telemetry.update();

		waitForStart();
		resetStartTime();

		telemetry.addLine("Starting...");
		phoneCam.setPipeline(pipeline);

		double[] tmpminValues = pipeline.minValues.val.clone();
		double[] tmpmaxValues = pipeline.maxValues.val.clone();
		float lrdrive, fbdrive, turn, extraAxis;

		lrdrive = gamepad1.left_stick_x;
//			fbdrive = -gamepad1.left_stick_y;
//			turn = gamepad1.right_stick_x;
//			extraAxis = -gamepad1.right_stick_y;
		for (int i= 0;Math.abs(lrdrive)>=0.75 && i<5;i++) {
			if (lrdrive>=0.75) {
				tmpminValues[0] = pipeline.minValues.val[0] + 5.0;
				tmpmaxValues[0] = pipeline.minValues.val[0] + 5.0;
				sleep(50);
			} else if (lrdrive<=-0.75) {
				tmpminValues[0] = pipeline.minValues.val[0] - 5.0;
				tmpmaxValues[0] = pipeline.minValues.val[0] - 5.0;
				sleep(50);
			}
			pipeline.minValues.set(tmpminValues);
			pipeline.minValues.set(tmpmaxValues);
		}
		while (opModeIsActive()) {
			telemetry.addData("Run Time", "%.5f", getRuntime());
			//telemetry.addData("Gamepad", "leftx (%.2f), lefty (%.2f), rightx (%.2f), righty (%.2f)", lrdrive, fbdrive, turn, extraAxis);
			// Display the Gamepad Stick info
//			telemetry.addLine("Gamepad" + "\r\n")
//					.addData("lrdrive", "%.3f", lrdrive)
//					.addData("fbdrive", "%.3f", fbdrive)
//					.addData("turn", "%.3f", turn)
//					.addData("extraAxis", "%.3f", extraAxis);
			telemetry.addLine("HSV Values:  ")
					.addData("Hue",pipeline.maxValues.val[0] + " - " + pipeline.minValues.val[0])
					.addData("Saturation",pipeline.maxValues.val[1] + " - " + pipeline.minValues.val[1])
					.addData("Value (Brightness)",pipeline.maxValues.val[2] + " - " + pipeline.minValues.val[2]);
			telemetry.update();

			sleep(100);
		}
	}

	class Pipeline extends OpenCvPipeline {
		// H ranges 0-180, S and V range 0-255
		Scalar maxValues = new Scalar(45, 255, 255);
		Scalar minValues = new Scalar(15, 180, 120);
		double approxDistance;
		boolean viewportPaused = false;

		Point center = new Point();
		List<MatOfPoint> contours = new ArrayList<>();
		MatOfPoint2f approxCurve, contour2f;
		MatOfPoint points;

		Mat output = new Mat(cameraRes,cameraType, new Scalar(0));
		Mat hsvimage = new Mat(cameraRes,cameraType, new Scalar(0));
		Mat thresh = new Mat(cameraRes,cameraType, new Scalar(0));
		Mat hierarchy = new Mat();
		Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, new Size(15, 15));
		Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_ERODE, new Size(5, 5));
		Mat blurredImage = new Mat();
		Rect boundingRect = new Rect();


		@Override
		public Mat processFrame(Mat input) {
			Imgproc.blur(input, blurredImage, new Size(5, 5));
			Imgproc.cvtColor(blurredImage, hsvimage, Imgproc.COLOR_RGB2HSV);
			Core.inRange(hsvimage,minValues,maxValues, thresh);

			Imgproc.erode(thresh, thresh, erodeElement);
			Imgproc.erode(thresh, thresh, erodeElement);
			Imgproc.dilate(thresh, thresh, dilateElement);
			Imgproc.dilate(thresh, thresh, dilateElement);

			contours = new ArrayList<>();
			Imgproc.findContours(thresh, contours, hierarchy,
					Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

			output.copySize(input);
			if (contours.size()>0) {
				for (int i = 0; i<contours.size(); i++) {
					// Minimum size allowed for consideration
					contour2f = new MatOfPoint2f(contours.get(i).toArray());

					//Processing on mMOP2f1 which is in type MatOfPoint2f
					approxCurve = new MatOfPoint2f();
					approxDistance = Imgproc.arcLength(contour2f, true)*0.04;
					Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

					//Convert back to MatOfPoint
					points = new MatOfPoint(approxCurve.toArray());

					// Get bounding rect of contour
					boundingRect = Imgproc.boundingRect(points);
					center = new Point((boundingRect.x+boundingRect.br().x)*0.5, (boundingRect.y+boundingRect.br().y)*0.5);
					Imgproc.drawMarker(output,center,new Scalar(0,100+(i*25),0, 100),
							Imgproc.MARKER_TILTED_CROSS);
					Imgproc.rectangle(output, new Point(boundingRect.x, boundingRect.y),
							new Point(boundingRect.x + boundingRect.width, boundingRect.y + boundingRect.height),
							new Scalar(0, 255, 0, 50), 1);
				}
			}

			return output;
		}

		@Override
		public void onViewportTapped() {
			viewportPaused = !viewportPaused;
			if(viewportPaused) phoneCam.pauseViewport();
			else phoneCam.resumeViewport();
		}
	}
}