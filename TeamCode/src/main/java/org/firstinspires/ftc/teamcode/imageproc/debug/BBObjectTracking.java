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

package org.firstinspires.ftc.teamcode.imageproc.debug;

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

@Autonomous(name="Object Tracking (BareBones)", group="Debug")
public class BBObjectTracking extends LinearOpMode {
	OpenCvCamera phoneCam;
	final Size cameraRes = new Size(320, 240);
	final int cameraType = 0;

	@Override
	public void runOpMode() {
		// Make an instance of the phone camera
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

		phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				// Start Streaming
				phoneCam.startStreaming((int) cameraRes.width, (int) cameraRes.height, OpenCvCameraRotation.UPRIGHT);
			}
		});

		waitForStart();
		resetStartTime();
		telemetry.addLine("Starting...");
		phoneCam.setPipeline(new Pipeline());

		while (opModeIsActive()) {
			if(gamepad1.a) {
				phoneCam.stopStreaming();
			}
			sleep(100);
		}
	}

	class Pipeline extends OpenCvPipeline {
		boolean viewportPaused = false;

		// H ranges 0-180, S and V range 0-255
		final Scalar maxValues = new Scalar(50, 255, 255);
		final Scalar minValues = new Scalar(15, 150, 150);
		double runtime = 0;
		double approxDistance;

		List<MatOfPoint> contours = new ArrayList<>();
		MatOfPoint2f approxCurve,contour2f;
		MatOfPoint points;

		Mat tmp = new Mat(cameraRes,cameraType, new Scalar(0));
		Mat output = new Mat(cameraRes,cameraType, new Scalar(0));
		Mat hsvimage = new Mat(cameraRes,cameraType, new Scalar(0));
		Mat thresh = new Mat(cameraRes,cameraType, new Scalar(0));
		Mat dst = new Mat(cameraRes,cameraType, new Scalar(0));
		Mat hierarchy = new Mat();
		Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15, 15));
		Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
		Rect rect;
		Mat blurredImage = new Mat();
		Mat mask = new Mat();
		Mat morphOutput = new Mat();


		@Override
		public Mat processFrame(Mat input) {
			Imgproc.blur(input, blurredImage, new Size(7, 7));
			Imgproc.cvtColor(blurredImage, hsvimage, Imgproc.COLOR_RGB2HSV);
			Core.inRange(hsvimage,minValues,maxValues, thresh);

			Imgproc.erode(thresh, thresh, erodeElement);
			Imgproc.erode(thresh, thresh, erodeElement);
			Imgproc.dilate(thresh, thresh, dilateElement);
			Imgproc.dilate(thresh, thresh, dilateElement);

			List<MatOfPoint> contours = new ArrayList<>();
			Imgproc.findContours(thresh, contours, hierarchy,
					Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

			input.copyTo(output);
			for (int contourIdx=0; contourIdx < contours.size(); contourIdx++) {
				// Minimum size allowed for consideration
				MatOfPoint2f approxCurve = new MatOfPoint2f();
				MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(contourIdx).toArray());

				//Processing on mMOP2f1 which is in type MatOfPoint2f
				double approxDistance = Imgproc.arcLength(contour2f, true)*0.02;
				Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

				//Convert back to MatOfPoint
				MatOfPoint points = new MatOfPoint(approxCurve.toArray());

				// Get bounding rect of contour
				Rect rect = Imgproc.boundingRect(points);

				Imgproc.rectangle(output, new Point(rect.x, rect.y),
						new Point(rect.x + rect.width, rect.y + rect.height),
						new Scalar(255, 0, 0, 100), 3);
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