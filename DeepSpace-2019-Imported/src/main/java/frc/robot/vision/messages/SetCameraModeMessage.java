package frc.robot.vision.messages;

/**
 * A Message that contains and can set the state of the camera and intake systems.
 */
public class SetCameraModeMessage extends VisionMessage {

    private static final String K_FRONT_CAMERA = "front";
    private static final String K_BACK_CAMERA = "back";

    private String mMessage = K_FRONT_CAMERA;

    public SetCameraModeMessage(String message) {
        mMessage = message;
    }

    public static SetCameraModeMessage getFrontCameraMessage() {
        return new SetCameraModeMessage(K_FRONT_CAMERA);
    }

    public static SetCameraModeMessage getBackCameraMessage() {
        return new SetCameraModeMessage(K_BACK_CAMERA);
    }

    @Override
    public String getType() {
        return "camera_mode";
    }

    @Override
    public String getMessage() {
        return mMessage;
    }
}
