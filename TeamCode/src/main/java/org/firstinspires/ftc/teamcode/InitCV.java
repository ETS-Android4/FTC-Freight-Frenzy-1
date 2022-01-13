package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.pipelines.DuckDetector;
//import org.firstinspires.ftc.teamcode.pipelines.PickupPosition;
import org.firstinspires.ftc.teamcode.pipelines.PickupPosition;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class InitCV {
    private OpenCvInternalCamera phoneCam;

    public void init(DuckDetector detector, int id) {
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, id);
        phoneCam.setPipeline(detector);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                // is a camera flashlight legal?
//                phoneCam.setFlashlightEnabled(true);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public void init(PickupPosition detector, int id) {
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, id);
        phoneCam.setPipeline(detector);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                // is a camera flashlight legal?
//                phoneCam.setFlashlightEnabled(true);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

//    public void init(PickupPosition detector, int id) {
//        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, id);
//        phoneCam.setPipeline(detector);
//        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//                // is a camera flashlight legal?
////                phoneCam.setFlashlightEnabled(true);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//    }

    public void stopStream() {
        phoneCam.stopStreaming();
    }
}
