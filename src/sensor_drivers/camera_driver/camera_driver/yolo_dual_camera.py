import cv2
import time
import threading
from ultralytics import YOLO


MODEL_PATH = "/workspace/saads_ws/yolo11n.pt"

LEFT_CAMERA = "/dev/video0"
RIGHT_CAMERA = "/dev/video2"

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 30

YOLO_IMAGE_SIZE = 640
CONFIDENCE_THRESHOLD = 0.25

DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 360


class CameraReader:
    def __init__(self, device, camera_name):
        self.device = device
        self.camera_name = camera_name

        self.cap = cv2.VideoCapture(
            device,
            cv2.CAP_V4L2
        )

        self.cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*"MJPG")
        )

        self.cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            CAMERA_WIDTH
        )

        self.cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            CAMERA_HEIGHT
        )

        self.cap.set(
            cv2.CAP_PROP_FPS,
            CAMERA_FPS
        )

        self.cap.set(
            cv2.CAP_PROP_BUFFERSIZE,
            1
        )

        if not self.cap.isOpened():
            raise RuntimeError(
                f"Could not open {camera_name} at {device}"
            )

        self.frame = None
        self.running = True

        self.thread = threading.Thread(
            target=self.update,
            daemon=True
        )

        self.thread.start()

        print(
            f"{camera_name} opened on {device}",
            flush=True
        )

    def update(self):
        while self.running:
            ret, frame = self.cap.read()

            if ret:
                # Always replace with newest frame.
                # Old frames do not build up.
                self.frame = frame

    def read(self):
        if self.frame is None:
            return None

        return self.frame.copy()

    def stop(self):
        self.running = False

        if self.thread.is_alive():
            self.thread.join(timeout=1.0)

        self.cap.release()


def print_detections(camera_name, result, model):

    objects = []

    for box in result.boxes:

        class_id = int(box.cls[0])
        confidence = float(box.conf[0])

        class_name = model.names[class_id]

        objects.append(
            f"{class_name} {confidence:.2f}"
        )

    if objects:
        print(
            f"{camera_name}: "
            + ", ".join(objects),
            flush=True
        )
    else:
        print(
            f"{camera_name}: no detections",
            flush=True
        )


def main():

    print("Loading YOLO model...", flush=True)

    model = YOLO(MODEL_PATH)

    print("Opening cameras...", flush=True)

    left_camera = CameraReader(
        LEFT_CAMERA,
        "FRONT LEFT"
    )

    right_camera = CameraReader(
        RIGHT_CAMERA,
        "FRONT RIGHT"
    )

    # Give both cameras time to begin producing frames.
    time.sleep(1)

    print(
        "Dual-camera computer vision running",
        flush=True
    )

    try:

        while True:

            left_frame = left_camera.read()
            right_frame = right_camera.read()

            if (
                left_frame is None
                or right_frame is None
            ):
                continue

            start_time = time.time()

            # Run the two images together as one YOLO batch.
            results = model(
                [
                    left_frame,
                    right_frame
                ],
                device=0,
                conf=CONFIDENCE_THRESHOLD,
                imgsz=YOLO_IMAGE_SIZE,
                verbose=False
            )

            elapsed = time.time() - start_time

            if elapsed > 0:
                batch_rate = 1.0 / elapsed
            else:
                batch_rate = 0.0

            left_result = results[0]
            right_result = results[1]

            print(
                f"\nPer-camera inference rate: "
                f"{batch_rate:.1f} FPS",
                flush=True
            )

            print_detections(
                "LEFT",
                left_result,
                model
            )

            print_detections(
                "RIGHT",
                right_result,
                model
            )

            # Draw YOLO detections.
            left_annotated = left_result.plot()
            right_annotated = right_result.plot()

            # Camera labels.
            cv2.putText(
                left_annotated,
                "FRONT LEFT",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2
            )

            cv2.putText(
                right_annotated,
                "FRONT RIGHT",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2
            )

            # Resize only for the display.
            left_display = cv2.resize(
                left_annotated,
                (
                    DISPLAY_WIDTH,
                    DISPLAY_HEIGHT
                )
            )

            right_display = cv2.resize(
                right_annotated,
                (
                    DISPLAY_WIDTH,
                    DISPLAY_HEIGHT
                )
            )

            # Put both camera views next to each other.
            combined = cv2.hconcat(
                [
                    left_display,
                    right_display
                ]
            )

            cv2.putText(
                combined,
                f"Per-camera inference: "
                f"{batch_rate:.1f} FPS",
                (20, DISPLAY_HEIGHT - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

            cv2.imshow(
                "SAADS Dual Front Camera Vision",
                combined
            )

            if (
                cv2.waitKey(1) & 0xFF
                == ord("q")
            ):
                break

    except KeyboardInterrupt:
        print(
            "\nStopping dual-camera vision...",
            flush=True
        )

    finally:

        left_camera.stop()
        right_camera.stop()

        cv2.destroyAllWindows()

        print(
            "Both cameras released",
            flush=True
        )


if __name__ == "__main__":
    main()
