"""
gemini_vision.py  —  Stream camera frames from ROS2 to Gemini Robotics ER 1.6.

Supports both sensor_msgs/Image (raw) and sensor_msgs/CompressedImage topics.
Draws persistent bounding boxes on detected lab equipment.
Publishes detection results to /gemini/detections (std_msgs/String, JSON).

Usage:
    # Simulation camera (raw Image):
    python gemini_vision.py --topic /camera_oak_gripper/rgb/image_raw

    # Real gripper camera (CompressedImage):
    python gemini_vision.py --topic /camera_oak_gripper/rgb/image_rect/compressed --compressed

    # Ask a specific question about the scene:
    python gemini_vision.py --topic /camera_oak_gripper/rgb/image_raw \
        --prompt "What Bruker instruments can you see?"

Requirements:
    export GEMINI_API_KEY=your_key_here
    pip install google-genai opencv-python
"""

import argparse
import json
import re
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from google import genai
from google.genai import types


MODEL_ID      = "gemini-robotics-er-1.6-preview"
ANALYSIS_HZ   = 0.2   # analyse one frame every 5 seconds

DEFAULT_PROMPT = (
    "You are assisting a lab robot. Identify all visible lab equipment or objects. "
    "For each item output EXACTLY one line in this format:\n"
    "[ymin, xmin, ymax, xmax]: label\n"
    "Coordinates are in the range 0-1000 (normalised). "
    "Do not add any other text."
)


class GeminiVisionNode(Node):

    def __init__(self, topic: str, compressed: bool, prompt: str):
        super().__init__("gemini_vision")

        import os
        api_key = os.environ.get("GEMINI_API_KEY", "")
        if not api_key:
            self.get_logger().error(
                "GEMINI_API_KEY not set. Run: export GEMINI_API_KEY=your_key"
            )
            raise SystemExit(1)

        self.client  = genai.Client(api_key=api_key)
        self.prompt  = prompt
        self.compressed = compressed

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        if compressed:
            self.sub = self.create_subscription(
                CompressedImage, topic, self._cb_compressed, qos
            )
        else:
            self.sub = self.create_subscription(
                Image, topic, self._cb_raw, qos
            )

        self.det_pub = self.create_publisher(String, "/gemini/detections", 10)

        self.last_t       = 0.0
        self.processing   = False
        self.detections   = []   # [[ymin,xmin,ymax,xmax,label], ...]

        self.get_logger().info(
            f"Gemini Vision ready  |  topic: {topic}  |  model: {MODEL_ID}"
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _cb_compressed(self, msg: CompressedImage):
        arr   = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is not None:
            self._process(frame)

    def _cb_raw(self, msg: Image):
        dtype  = np.uint8
        arr    = np.frombuffer(msg.data, dtype=dtype)
        # Handle common encodings
        if msg.encoding in ("rgb8", "bgr8", "rgba8", "bgra8"):
            channels = 4 if msg.encoding.endswith("a8") else 3
            frame = arr.reshape((msg.height, msg.width, channels))
            if msg.encoding.startswith("rgb"):
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            frame = arr.reshape((msg.height, msg.width))
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        self._process(frame)

    # ------------------------------------------------------------------
    # Core logic
    # ------------------------------------------------------------------

    def _process(self, frame: np.ndarray):
        now = time.time()
        if (now - self.last_t > 1.0 / ANALYSIS_HZ) and not self.processing:
            self.last_t = now
            self._query_gemini(frame.copy())

        self._draw(frame)
        cv2.imshow("Gemini Robotics Vision", frame)
        cv2.waitKey(1)

    def _query_gemini(self, frame: np.ndarray):
        self.processing = True
        try:
            _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            response = self.client.models.generate_content(
                model=MODEL_ID,
                contents=[
                    self.prompt,
                    types.Part.from_bytes(data=buf.tobytes(), mime_type="image/jpeg"),
                ],
            )
            parsed = self._parse(response.text)
            if parsed:
                self.detections = parsed
                self.get_logger().info(f"Detected {len(parsed)} item(s).")
                self._publish_detections(parsed)
        except Exception as exc:
            self.get_logger().warn(f"Gemini error: {exc}")
        finally:
            self.processing = False

    def _parse(self, text: str):
        pattern = r"\[(\d+)[,\s]+(\d+)[,\s]+(\d+)[,\s]+(\d+)\]\s*[:\-]?\s*(.+)"
        results = []
        for m in re.finditer(pattern, text):
            try:
                coords = [int(m.group(i)) for i in range(1, 5)]
                label  = m.group(5).strip().split("\n")[0].upper()
                results.append(coords + [label])
            except Exception:
                continue
        return results

    def _draw(self, frame: np.ndarray):
        h, w = frame.shape[:2]

        if self.processing:
            cv2.putText(frame, "AI THINKING...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        for det in self.detections:
            ymin, xmin, ymax, xmax, label = det
            x1 = int(xmin * w / 1000)
            y1 = int(ymin * h / 1000)
            x2 = int(xmax * w / 1000)
            y2 = int(ymax * h / 1000)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 80), 2)

            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
            cv2.rectangle(frame, (x1, y1 - th - 8), (x1 + tw + 8, y1), (0, 220, 80), -1)
            cv2.putText(frame, label, (x1 + 4, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2)

    def _publish_detections(self, detections):
        payload = [
            {"ymin": d[0], "xmin": d[1], "ymax": d[2], "xmax": d[3], "label": d[4]}
            for d in detections
        ]
        msg      = String()
        msg.data = json.dumps(payload)
        self.det_pub.publish(msg)


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Gemini Robotics vision node")
    parser.add_argument(
        "--topic", default="/camera_oak_gripper/rgb/image_raw",
        help="ROS2 camera topic to subscribe to"
    )
    parser.add_argument(
        "--compressed", action="store_true",
        help="Topic publishes CompressedImage instead of Image"
    )
    parser.add_argument(
        "--prompt", default=DEFAULT_PROMPT,
        help="Custom prompt sent to Gemini with each frame"
    )
    args = parser.parse_args()

    rclpy.init()
    try:
        node = GeminiVisionNode(
            topic=args.topic,
            compressed=args.compressed,
            prompt=args.prompt,
        )
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
