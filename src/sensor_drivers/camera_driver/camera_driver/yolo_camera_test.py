import cv2
import time
import threading
from ultralytics import YOLO

MODEL_PATH = "/workspace/saads_ws/yolo11n.pt"
CAMERA_DEVICE = "/dev/video0"

CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
CAMERA_FPS = 10

DISPLAY_WIDTH = 1280
DISPLAY_HEIGHT = 720

CONFIDENCE_THRESHOLD = 0.25


class CameraReader:
    def __init__(self, device):
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)

        self.cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*"MJPG")
        )

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open {device}")

        self.ret = False
        self.frame = None
        self.running = True

        self.thread = threading.Thread(
            target=self.update,
            daemon=True
        )

        self.thread.start()

    def update(self):
        while self.running:
            ret, frame = self.cap.read()

            if ret:
                self.ret = ret
                self.frame = frame

    def read(self):
        return self.ret, self.frame

    def stop(self):
        self.running = False
        self.thread.join()
        self.cap.release()


def main():

    print("Loading YOLO model...")
    model = YOLO(MODEL_PATH)

    print("Opening camera...")
    camera = CameraReader(CAMERA_DEVICE)

    time.sleep(1)

    print("Running YOLO")
    print("Press q to quit")

    try:
        while True:

            ret, frame = camera.read()

            if not ret or frame is None:
                continue

            # Copy newest frame so capture thread can continue
            frame = frame.copy()

            start = time.time()

            results = model(
                frame,
                device=0,
                conf=CONFIDENCE_THRESHOLD,
                imgsz=640,
                verbose=False
            )

            inference_time = time.time() - start

            fps = (
                1.0 / inference_time
                if inference_time > 0
                else 0
            )

            result = results[0]

            print(
                f"Inference: {fps:.1f} FPS | "
                f"Objects: {len(result.boxes)}"
            )

            annotated = result.plot()

            cv2.putText(
                annotated,
                f"Inference FPS: {fps:.1f}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2
            )

            display_frame = cv2.resize(
                annotated,
                (DISPLAY_WIDTH, DISPLAY_HEIGHT)
            )

            cv2.imshow(
                "SAADS YOLO Camera",
                display_frame
            )

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        camera.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
