package org.firstinspires.ftc.teamcode.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PickupPosition extends OpenCvPipeline {
    Mat THRESHOLD = new Mat();
    Mat MATRIX_BOX = new Mat();
    Mat contoursOnFrame = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();
    int numContoursFound;
    private Telemetry telemetry = null;

    enum Stage {
        YCbCr_CHAN2,
        THRESHOLD,
        CONTOURS_OVERLAYED_ON_FRAME,
        RAW_IMAGE,
    }

    private Stage stageToRenderToViewport = Stage.YCbCr_CHAN2;
    private Stage[] stages = Stage.values();

    public PickupPosition(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void onViewportTapped() {
        int currentStageNum = stageToRenderToViewport.ordinal();

        int nextStageNum = currentStageNum + 1;

        if (nextStageNum >= stages.length) {
            nextStageNum = 0;
        }

        stageToRenderToViewport = stages[nextStageNum];
    }

    @Override
    public Mat processFrame(Mat input)
    {
        contoursList.clear();
        Imgproc.cvtColor(input, MATRIX_BOX, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(MATRIX_BOX, MATRIX_BOX, 1);
        Imgproc.threshold(MATRIX_BOX, THRESHOLD, 150, 255, Imgproc.THRESH_BINARY);
        Imgproc.findContours(THRESHOLD, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        numContoursFound = contoursList.size();
        input.copyTo(contoursOnFrame);
        Imgproc.drawContours(contoursOnFrame, contoursList, -1, new Scalar(0, 0, 255), 3, 8);
        telemetry.addData("Total Contours", numContoursFound);
        telemetry.update();


        List<Rect> contourRects = new ArrayList();
        for (Mat contour : contoursList) {
            double area = Imgproc.contourArea(contour);
            double sqrtArea = Math.sqrt(area);
            telemetry.addData("Area", area);

//            todo: add condition that checks if there is already a contour rectangle found
//            todo: only one should be included at max
//            && sqrtArea < 90
            if (sqrtArea > 20) {
                contourRects.add(Imgproc.boundingRect(contour));
            }
        }

        if (contourRects.size() == 1) {
            telemetry.addData("Found blocks", contourRects.size());
            telemetry.update();
        }

        for (int i=0; i < contourRects.size(); i++) {
            Imgproc.rectangle(contoursOnFrame, contourRects.get(i), new Scalar(255, 0, 0), 2);
            double halfwayPoint = contourRects.get(i).x + .5 * contourRects.get(i).width;
            double centerOffset = input.cols() / 2 - halfwayPoint;

            Imgproc.putText(
                    contoursOnFrame,
                    Integer.toString(contourRects.size()),
                    new Point(
                            30,
                            50),
                    Imgproc.FONT_HERSHEY_PLAIN,
                    3,
                    new Scalar(128, 0, 128),
                    6);

            telemetry.addData("Pixels to rotate", centerOffset);
        }

        switch (stageToRenderToViewport) {
            case YCbCr_CHAN2: {
                return MATRIX_BOX;
            }

            case THRESHOLD: {
                return THRESHOLD;
            }

            case CONTOURS_OVERLAYED_ON_FRAME: {
                return contoursOnFrame;
            }

            default: {
                return input;
            }
        }
    }

}