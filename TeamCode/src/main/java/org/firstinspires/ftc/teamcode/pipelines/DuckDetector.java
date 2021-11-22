package org.firstinspires.ftc.teamcode.pipelines;
import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;

public class DuckDetector extends OpenCvPipeline{
    double LEFT_value;
    double RIGHT_value;
    double MID_value;
    private String location = "";

    Mat mat = new Mat();

    private String[] LOCATION = {"LEFT", "RIGHT", "MIDDLE", "NOT FOUND"};

    static final Rect LEFT_MAT = new Rect(
            15,
            110,
            50,
            120
    );

    static final Rect MID_MAT = new Rect(
            95,
            110,
            50,
            120
    );

    static final Rect RIGHT_MAT = new Rect(
            170,
            110,
            50,
            120
    );

    final static double PCT = 0;

    Mat LEFT;
    Mat RIGHT;
    Mat MID;

    public DuckDetector() {}

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        double low[] = {23, 50, 70};
        double high[] = {32, 255, 255};

        final Scalar LOW_BOUND = new Scalar(low[0], low[1], low[2]);
        final Scalar UPPER_BOUND = new Scalar(high[0], high[1], high[2]);

        Core.inRange(mat, LOW_BOUND, UPPER_BOUND, mat);

        LEFT = mat.submat(LEFT_MAT);
        RIGHT = mat.submat(RIGHT_MAT);
        MID = mat.submat(MID_MAT);

        LEFT_value = Core.sumElems(LEFT).val[0] / LEFT_MAT.area() / 255;
        MID_value = Core.sumElems(MID).val[0] / MID_MAT.area() / 255;
        RIGHT_value = Core.sumElems(RIGHT).val[0] / RIGHT_MAT.area() / 255;

        LEFT.release();
        MID.release();
        RIGHT.release();

        boolean dl = ((LEFT_value > MID_value) && (LEFT_value > RIGHT_value));
        boolean dm = ((MID_value > LEFT_value) && (MID_value > RIGHT_value));
        boolean dr = ((RIGHT_value > LEFT_value) && (RIGHT_value > MID_value));

        if (dl) {
            location = LOCATION[0];
        } else if (dr) {
            location = LOCATION[1];
        } else if (dm) {
            location = LOCATION[2];
        } else {
            location = LOCATION[3];
        }

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar duckLocationFound = new Scalar(0, 255, 0);
        Scalar duckNotFound = new Scalar(255, 0, 0);

        Imgproc.rectangle(mat, LEFT_MAT, location == LOCATION[0] ? duckLocationFound:duckNotFound);
        Imgproc.rectangle(mat, RIGHT_MAT, location == LOCATION[1] ? duckLocationFound:duckNotFound);
        Imgproc.rectangle(mat, MID_MAT, location == LOCATION[2] ? duckLocationFound:duckNotFound);

        return mat;
    }

    public String getLoc() { return location; }

    public double[] getLR() {
        return new double[]{Core.sumElems(LEFT).val[0], Core.sumElems(MID).val[0], Core.sumElems(RIGHT).val[0]};
    }

    public Long[] getConf() {
        return new Long[]{Math.round(LEFT_value * 100), Math.round(MID_value * 100), Math.round(RIGHT_value * 100)};
    }
}
