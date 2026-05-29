#!/usr/bin/env python3
import os
import time
from typing import Optional, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image, CompressedImage

from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_detector")

        # ----------------------------
        # Input parameters
        # ----------------------------
        self.declare_parameter("use_compressed", True)
        self.declare_parameter("image_topic", "/zed/zed_node/rgb/color/rect/image")
        self.declare_parameter(
            "compressed_topic", "/zed/zed_node/rgb/color/rect/image/compressed"
        )

        # ----------------------------
        # Output parameters
        # ----------------------------
        self.declare_parameter("ids_topic", "/aruco/ids")

        # IMPORTANT: keep this name stable for RViz
        # We will publish the ANNOTATED image here, either as raw or compressed depending on params.
        self.declare_parameter("annotated_topic", "/aruco/annotated_image")

        # Choose annotated output type(s)
        self.declare_parameter(
            "publish_annotated_compressed", True
        )  # recommended for HaLow
        self.declare_parameter("publish_annotated_raw", False)  # only for local debug
        self.declare_parameter("annotated_raw_topic", "/aruco/annotated_image/raw")
        self.declare_parameter("annotated_jpeg_quality", 35)  # 1..100

        # ----------------------------
        # Saving parameters
        # ----------------------------
        self.declare_parameter("save_dir", os.path.expanduser("~/aruco_captures"))
        self.declare_parameter("save_mode", "crop")  # 'crop' or 'full'
        self.declare_parameter("crop_padding_px", 20)
        self.declare_parameter("overwrite_latest_per_id", True)
        self.declare_parameter("also_save_timestamped", True)
        self.declare_parameter("min_save_interval_sec", 0.5)  # per ID

        # ----------------------------
        # ArUco parameters
        # ----------------------------
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")

        # Read params
        self.use_compressed = bool(self.get_parameter("use_compressed").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.compressed_topic = str(self.get_parameter("compressed_topic").value)

        self.ids_topic = str(self.get_parameter("ids_topic").value)
        self.annotated_topic = str(self.get_parameter("annotated_topic").value)

        self.publish_annotated_compressed = bool(
            self.get_parameter("publish_annotated_compressed").value
        )
        self.publish_annotated_raw = bool(
            self.get_parameter("publish_annotated_raw").value
        )
        self.annotated_raw_topic = str(self.get_parameter("annotated_raw_topic").value)
        self.annotated_jpeg_quality = int(
            self.get_parameter("annotated_jpeg_quality").value
        )

        self.save_dir = str(self.get_parameter("save_dir").value)
        self.save_mode = str(self.get_parameter("save_mode").value).strip().lower()
        self.crop_padding_px = int(self.get_parameter("crop_padding_px").value)
        self.overwrite_latest_per_id = bool(
            self.get_parameter("overwrite_latest_per_id").value
        )
        self.also_save_timestamped = bool(
            self.get_parameter("also_save_timestamped").value
        )
        self.min_save_interval_sec = float(
            self.get_parameter("min_save_interval_sec").value
        )

        dict_name = str(self.get_parameter("aruco_dictionary").value).strip()
        self.aruco_dict = self._load_aruco_dict(dict_name)

        # ArUco detector params (distance-friendly)
        self.detector_params = cv2.aruco.DetectorParameters_create()
        self.detector_params.adaptiveThreshWinSizeMin = 3
        self.detector_params.adaptiveThreshWinSizeMax = 53
        self.detector_params.adaptiveThreshWinSizeStep = 4
        self.detector_params.minMarkerPerimeterRate = 0.01
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector_params.cornerRefinementWinSize = 5
        self.detector_params.cornerRefinementMaxIterations = 30
        self.detector_params.cornerRefinementMinAccuracy = 0.05
        self.detector_params.perspectiveRemovePixelPerCell = 8
        self.detector_params.perspectiveRemoveIgnoredMarginPerCell = 0.13
        self.detector_params.errorCorrectionRate = 0.8

        self.bridge = CvBridge()

        # QoS: large sensor data -> BEST_EFFORT and depth=1
        self.image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.pub_ids = self.create_publisher(Int32MultiArray, self.ids_topic, 10)

        # Annotated publishers
        self.pub_annotated_compressed = None
        self.pub_annotated_raw = None

        # Compressed annotated goes to annotated_topic (stable for RViz)
        if self.publish_annotated_compressed:
            self.pub_annotated_compressed = self.create_publisher(
                CompressedImage, self.annotated_topic, 1
            )

        # Optional raw annotated goes to a separate topic
        if self.publish_annotated_raw:
            self.pub_annotated_raw = self.create_publisher(
                Image, self.annotated_raw_topic, 1
            )

        # Subscriber (input image)
        if self.use_compressed:
            self.sub = self.create_subscription(
                CompressedImage,
                self.compressed_topic,
                self._on_compressed,
                self.image_qos,
            )
            self.get_logger().info(
                f"Subscribed to CompressedImage: {self.compressed_topic}"
            )
        else:
            self.sub = self.create_subscription(
                Image, self.image_topic, self._on_raw, self.image_qos
            )
            self.get_logger().info(f"Subscribed to Image: {self.image_topic}")

        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f"Saving captures under: {self.save_dir}")

        self.last_save_time_sec: Dict[int, float] = {}

        self.get_logger().info(f"Using ArUco dictionary: {dict_name}")
        if self.pub_annotated_compressed is not None:
            self.get_logger().info(
                f"Publishing annotated as CompressedImage on: {self.annotated_topic} (jpeg_quality={self.annotated_jpeg_quality})"
            )
        if self.pub_annotated_raw is not None:
            self.get_logger().warn(
                f"Publishing annotated RAW on: {self.annotated_raw_topic} (high bandwidth)"
            )

    def _load_aruco_dict(self, dict_name: str):
        if not hasattr(cv2.aruco, dict_name):
            self.get_logger().warn(
                f"Invalid aruco_dictionary '{dict_name}', falling back to DICT_4X4_50"
            )
            dict_name = "DICT_4X4_50"
        dict_id = getattr(cv2.aruco, dict_name)
        return cv2.aruco.Dictionary_get(dict_id)

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _on_raw(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge raw convert failed: {e}")
            return
        self._process_frame(
            frame, stamp_sec=self._stamp_to_sec(msg.header.stamp), header=msg.header
        )

    def _on_compressed(self, msg: CompressedImage) -> None:
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            if frame is None:
                self.get_logger().warn("cv2.imdecode returned None")
                return
        except Exception as e:
            self.get_logger().warn(f"Compressed decode failed: {e}")
            return
        self._process_frame(
            frame, stamp_sec=self._stamp_to_sec(msg.header.stamp), header=msg.header
        )

    def _to_bgr(self, frame: np.ndarray) -> Optional[np.ndarray]:
        if frame.ndim != 3:
            return None
        if frame.shape[2] == 3:
            return frame
        if frame.shape[2] == 4:
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return None

    def _publish_annotated(self, annotated_bgr: np.ndarray, header) -> None:
        # Compressed annotated on annotated_topic (stable for RViz)
        if self.pub_annotated_compressed is not None:
            try:
                q = self.annotated_jpeg_quality
                if q < 1:
                    q = 1
                elif q > 100:
                    q = 100

                ok, buf = cv2.imencode(
                    ".jpg", annotated_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(q)]
                )
                if not ok:
                    self.get_logger().warn("cv2.imencode(.jpg) failed")
                else:
                    out = CompressedImage()
                    out.header = header
                    out.format = "jpeg"
                    out.data = buf.tobytes()
                    self.pub_annotated_compressed.publish(out)
            except Exception as e:
                self.get_logger().warn(f"Publish annotated compressed failed: {e}")

        # Optional raw annotated on a different topic
        if self.pub_annotated_raw is not None:
            try:
                out_raw = self.bridge.cv2_to_imgmsg(annotated_bgr, encoding="bgr8")
                out_raw.header = header
                self.pub_annotated_raw.publish(out_raw)
            except Exception as e:
                self.get_logger().warn(f"Publish annotated raw failed: {e}")

    def _process_frame(self, frame: np.ndarray, stamp_sec: float, header) -> None:
        bgr = self._to_bgr(frame)
        if bgr is None:
            self.get_logger().warn(f"Unsupported frame shape: {frame.shape}")
            return

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)

        corners, ids, _rej = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.detector_params
        )

        annotated = bgr.copy()
        detected_ids = []

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten().astype(int).tolist()
            detected_ids = ids_flat
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            self._maybe_save(bgr, corners, ids_flat, stamp_sec)

        msg_ids = Int32MultiArray()
        msg_ids.data = detected_ids
        self.pub_ids.publish(msg_ids)

        self._publish_annotated(annotated, header)

    def _maybe_save(self, bgr: np.ndarray, corners, ids_flat, stamp_sec: float) -> None:
        t_now = time.time()

        for idx, marker_id in enumerate(ids_flat):
            last_t = float(self.last_save_time_sec.get(marker_id, 0.0))
            if (t_now - last_t) < self.min_save_interval_sec:
                continue
            self.last_save_time_sec[marker_id] = t_now

            if self.save_mode == "crop":
                crop = self._crop_marker(bgr, corners[idx], self.crop_padding_px)
                if crop is None:
                    continue
                img_to_save = crop
            else:
                img_to_save = bgr

            latest_path = os.path.join(self.save_dir, f"{marker_id}.png")
            if self.overwrite_latest_per_id:
                cv2.imwrite(latest_path, img_to_save)

            if self.also_save_timestamped:
                stamp_str = f"{stamp_sec:.3f}".replace(".", "_")
                ts_path = os.path.join(self.save_dir, f"{marker_id}_{stamp_str}.png")
                cv2.imwrite(ts_path, img_to_save)

    def _crop_marker(
        self, bgr: np.ndarray, corner_set: np.ndarray, pad: int
    ) -> Optional[np.ndarray]:
        pts = corner_set.reshape(-1, 2)
        xs = pts[:, 0]
        ys = pts[:, 1]

        x0 = int(max(float(np.min(xs)) - float(pad), 0.0))
        y0 = int(max(float(np.min(ys)) - float(pad), 0.0))
        x1 = int(min(float(np.max(xs)) + float(pad), float(bgr.shape[1] - 1)))
        y1 = int(min(float(np.max(ys)) + float(pad), float(bgr.shape[0] - 1)))

        if x1 <= x0 or y1 <= y0:
            return None
        return bgr[y0:y1, x0:x1].copy()


def main() -> None:
    rclpy.init()
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
