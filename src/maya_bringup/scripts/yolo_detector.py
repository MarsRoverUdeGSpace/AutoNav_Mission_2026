#!/usr/bin/env python3
import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from ultralytics import YOLO


class YoloDetector(Node):
    def __init__(self) -> None:
        super().__init__("yolo_detector")

        self.declare_parameter("use_compressed", True)
        self.declare_parameter("compressed_topic", "/zed/zed_node/rgb/color/rect/image/compressed")
        self.declare_parameter("model_path", "")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("conf_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("annotated_topic", "/yolo/annotated_image")
        self.declare_parameter("annotated_jpeg_quality", 35)
        self.declare_parameter("publish_annotated_compressed", True)
        self.declare_parameter("publish_annotated_raw", False)
        self.declare_parameter("annotated_raw_topic", "/yolo/annotated_image/raw")
        self.declare_parameter("detections_topic", "/yolo/detections_json")
        self.declare_parameter("process_every_n", 1)

        self.compressed_topic = str(self.get_parameter("compressed_topic").value)
        model_path = str(self.get_parameter("model_path").value).strip()
        self.device = str(self.get_parameter("device").value).strip()
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.annotated_topic = str(self.get_parameter("annotated_topic").value)
        self.annotated_jpeg_quality = int(self.get_parameter("annotated_jpeg_quality").value)
        self.publish_annotated_compressed = bool(self.get_parameter("publish_annotated_compressed").value)
        self.publish_annotated_raw = bool(self.get_parameter("publish_annotated_raw").value)
        self.annotated_raw_topic = str(self.get_parameter("annotated_raw_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.process_every_n = max(1, int(self.get_parameter("process_every_n").value))
        self.frame_count = 0
        self.bridge = CvBridge()

        if not model_path:
            self.get_logger().error("Parameter 'model_path' is empty. Set it to a .pt file.")
            raise RuntimeError("Missing model_path")
        if not Path(model_path).exists():
            self.get_logger().error(f"YOLO model not found: {model_path}")
            raise RuntimeError("Invalid model_path")

        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"Loaded YOLO model: {model_path}")
        except Exception as exc:
            self.get_logger().error(f"Failed to load YOLO model: {exc}")
            raise

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(
            CompressedImage, self.compressed_topic, self._on_image, qos
        )
        self.pub_annotated_compressed = None
        self.pub_annotated_raw = None
        if self.publish_annotated_compressed:
            self.pub_annotated_compressed = self.create_publisher(
                CompressedImage, self.annotated_topic, 1
            )
        if self.publish_annotated_raw:
            self.pub_annotated_raw = self.create_publisher(
                Image, self.annotated_raw_topic, 1
            )
        self.pub_detections = self.create_publisher(String, self.detections_topic, 10)

        self.get_logger().info(f"Subscribed to: {self.compressed_topic}")
        if self.pub_annotated_compressed is not None:
            self.get_logger().info(f"Publishing annotated compressed: {self.annotated_topic}")
        if self.pub_annotated_raw is not None:
            self.get_logger().info(f"Publishing annotated raw: {self.annotated_raw_topic}")
        self.get_logger().info(f"Publishing detections: {self.detections_topic}")

    def _on_image(self, msg: CompressedImage) -> None:
        self.frame_count += 1
        if (self.frame_count % self.process_every_n) != 0:
            return

        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn("Failed to decode compressed image")
            return

        try:
            results = self.model(
                frame,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                device=self.device,
                verbose=False,
            )
        except Exception as exc:
            self.get_logger().warn(f"YOLO inference failed: {exc}")
            return

        result = results[0]
        annotated = result.plot()

        if self.pub_annotated_compressed is not None:
            q = int(np.clip(self.annotated_jpeg_quality, 1, 100))
            ok, buf = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), q])
            if ok:
                out_img = CompressedImage()
                out_img.header = msg.header
                out_img.format = "jpeg"
                out_img.data = buf.tobytes()
                self.pub_annotated_compressed.publish(out_img)
        if self.pub_annotated_raw is not None:
            out_raw = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out_raw.header = msg.header
            self.pub_annotated_raw.publish(out_raw)

        dets = []
        names = result.names if result.names is not None else {}
        if result.boxes is not None:
            for b in result.boxes:
                xyxy = b.xyxy[0].tolist()
                cls_id = int(b.cls[0].item()) if b.cls is not None else -1
                conf = float(b.conf[0].item()) if b.conf is not None else 0.0
                dets.append(
                    {
                        "class_id": cls_id,
                        "class_name": names.get(cls_id, str(cls_id)),
                        "confidence": conf,
                        "bbox_xyxy": xyxy,
                    }
                )

        out_det = String()
        out_det.data = json.dumps(
            {
                "stamp_sec": int(msg.header.stamp.sec),
                "stamp_nanosec": int(msg.header.stamp.nanosec),
                "frame_id": msg.header.frame_id,
                "count": len(dets),
                "detections": dets,
            }
        )
        self.pub_detections.publish(out_det)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
