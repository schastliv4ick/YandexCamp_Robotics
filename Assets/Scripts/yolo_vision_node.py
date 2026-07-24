"""YOLO -> Unity UDP bridge for ball detection."""

from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import cv2
from ultralytics import YOLO

STREAM_URL = "http://192.168.2.156:8080"
MODEL_NAME = "../Models/YOLO/best_detect.pt"
CONFIDENCE = 0.20
TARGET_CLASSES = [0]

UDP_IP = "127.0.0.1"
UDP_PORT = 5005
PROTOCOL_VERSION = 1
STREAM_FRAME_TIMEOUT_SECONDS = 0.5
LOG_INTERVAL_SECONDS = 0.5


@dataclass(frozen=True)
class CapturedFrame:
    image: object
    sequence: int
    timestamp: float
    arrival_monotonic: float


latest_frame: Optional[CapturedFrame] = None
frame_lock = threading.Lock()
stop_event = threading.Event()


def capture_thread(cap: cv2.VideoCapture) -> None:
    """Keep only the newest camera frame so inference never builds a backlog."""
    global latest_frame

    sequence = 0
    while not stop_event.is_set():
        ok, frame = cap.read()
        now_monotonic = time.monotonic()

        if ok and frame is not None:
            captured = CapturedFrame(
                image=frame,
                sequence=sequence,
                timestamp=time.time(),
                arrival_monotonic=now_monotonic,
            )
            sequence = (sequence + 1) & 0x7FFFFFFF
            with frame_lock:
                latest_frame = captured
            continue

        with frame_lock:
            if (
                latest_frame is not None
                and now_monotonic - latest_frame.arrival_monotonic
                > STREAM_FRAME_TIMEOUT_SECONDS
            ):
                latest_frame = None
        time.sleep(0.01)


def pick_best_box(results) -> Tuple[Optional[Tuple[float, float, float, float]], float]:
    """Choose the most confident ball detection, using area as a tie-breaker."""
    if not results or results[0].boxes is None:
        return None, 0.0

    best_box = None
    best_confidence = 0.0
    best_area = 0.0

    for box in results[0].boxes:
        confidence = float(box.conf[0].cpu().item())
        x1, y1, x2, y2 = (float(value) for value in box.xyxy[0].cpu().tolist())
        area = max(0.0, x2 - x1) * max(0.0, y2 - y1)

        if confidence > best_confidence or (
            abs(confidence - best_confidence) < 1e-6 and area > best_area
        ):
            best_box = (x1, y1, x2, y2)
            best_confidence = confidence
            best_area = area

    return best_box, best_confidence


def make_packet(
    frame_sequence: int,
    frame_timestamp: float,
    frame_width: int,
    frame_height: int,
    box: Optional[Sequence[float]],
    confidence: float,
) -> dict:
    if box is None:
        return {
            "version": PROTOCOL_VERSION,
            "seq": frame_sequence,
            "timestamp": frame_timestamp,
            "frame_w": frame_width,
            "frame_h": frame_height,
            "angle": 0.0,
            "distance": 0.0,
            "sees": 0.0,
            "conf": 0.0,
            "w": 0.0,
            "h": 0.0,
            "cx": 0.5,
            "cy": 0.5,
            "clipped": False,
        }

    x1, y1, x2, y2 = (float(value) for value in box)
    x1 = min(max(x1, 0.0), float(frame_width))
    x2 = min(max(x2, 0.0), float(frame_width))
    y1 = min(max(y1, 0.0), float(frame_height))
    y2 = min(max(y2, 0.0), float(frame_height))

    width_pixels = max(0.0, x2 - x1)
    height_pixels = max(0.0, y2 - y1)
    center_x = 0.5 * (x1 + x2)
    center_y = 0.5 * (y1 + y2)

    width_normalized = width_pixels / frame_width
    height_normalized = height_pixels / frame_height
    center_x_normalized = center_x / frame_width
    center_y_normalized = center_y / frame_height
    angle = 2.0 * center_x_normalized - 1.0

    edge_margin = 1.0
    clipped = (
        x1 <= edge_margin
        or y1 <= edge_margin
        or x2 >= frame_width - edge_margin
        or y2 >= frame_height - edge_margin
    )

    return {
        "version": PROTOCOL_VERSION,
        "seq": frame_sequence,
        "timestamp": frame_timestamp,
        "frame_w": frame_width,
        "frame_h": frame_height,
        "angle": float(max(-1.0, min(1.0, angle))),
        "distance": float(max(0.0, min(1.0, height_normalized))),
        "sees": 1.0,
        "conf": float(max(0.0, min(1.0, confidence))),
        "w": float(max(0.0, min(1.0, width_normalized))),
        "h": float(max(0.0, min(1.0, height_normalized))),
        "cx": float(max(0.0, min(1.0, center_x_normalized))),
        "cy": float(max(0.0, min(1.0, center_y_normalized))),
        "clipped": bool(clipped),
    }


def main() -> None:
    global latest_frame

    stop_event.clear()
    with frame_lock:
        latest_frame = None

    print(f"[yolo_vision_node] Loading model: {MODEL_NAME}")
    model = YOLO(MODEL_NAME)
    print(f"[yolo_vision_node] Classes: {model.names}")

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    capture = cv2.VideoCapture(STREAM_URL)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not capture.isOpened():
        print(f"[yolo_vision_node] Cannot open stream: {STREAM_URL}")
        return

    reader = threading.Thread(target=capture_thread, args=(capture,), daemon=True)
    reader.start()

    last_processed_sequence = -1
    last_log_time = 0.0
    previous_inference_time = time.monotonic()
    smoothed_fps = 0.0

    print(
        f"[yolo_vision_node] Sending protocol v{PROTOCOL_VERSION} "
        f"to {UDP_IP}:{UDP_PORT}. Press q to stop."
    )

    try:
        while not stop_event.is_set():
            with frame_lock:
                captured = latest_frame
                if captured is not None and captured.sequence != last_processed_sequence:
                    frame = captured.image.copy()
                else:
                    frame = None

            if captured is None or frame is None:
                time.sleep(0.002)
                continue

            last_processed_sequence = captured.sequence
            frame_height, frame_width = frame.shape[:2]

            # Use actual detections rather than tracker-only predictions: the Unity
            # one-second handoff timer must represent continuous visual evidence.
            results = model.predict(
                frame,
                conf=CONFIDENCE,
                classes=TARGET_CLASSES,
                verbose=False,
            )
            best_box, confidence = pick_best_box(results)
            packet = make_packet(
                captured.sequence,
                captured.timestamp,
                frame_width,
                frame_height,
                best_box,
                confidence,
            )
            udp_socket.sendto(
                json.dumps(packet, separators=(",", ":")).encode("utf-8"),
                (UDP_IP, UDP_PORT),
            )

            if best_box is not None:
                x1, y1, x2, y2 = (int(value) for value in best_box)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    f"ball {confidence:.2f}",
                    (x1, max(0, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

            now = time.monotonic()
            delta = now - previous_inference_time
            previous_inference_time = now
            if delta > 0.0:
                instantaneous_fps = 1.0 / delta
                smoothed_fps = (
                    instantaneous_fps
                    if smoothed_fps <= 0.0
                    else 0.9 * smoothed_fps + 0.1 * instantaneous_fps
                )

            cv2.putText(
                frame,
                f"FPS: {smoothed_fps:.1f}",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )

            if now - last_log_time >= LOG_INTERVAL_SECONDS:
                last_log_time = now
                print(
                    "[UDP -> Unity] "
                    f"seq={packet['seq']} sees={int(packet['sees'])} "
                    f"angle={packet['angle']:+.3f} "
                    f"size={packet['w']:.3f}x{packet['h']:.3f} "
                    f"conf={packet['conf']:.2f} clipped={packet['clipped']}"
                )

            cv2.imshow("GFS-X YOLO Vision", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        stop_event.set()
        reader.join(timeout=1.0)
        capture.release()
        udp_socket.close()
        cv2.destroyAllWindows()
        print("[yolo_vision_node] Stopped.")


if __name__ == "__main__":
    main()
