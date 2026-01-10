#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import numpy as np
import cv2
import onnxruntime as ort

class TargetPublisher(Node):
    def __init__(self, node_name: str = "target_publisher"):
        super().__init__(node_name)

        self.declare_parameter("weights", "best.onnx")   # ONNX file
        self.declare_parameter("conf", 0.3)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("fov_deg", 40.35)
        self.declare_parameter("imgsz", 640)

        weights_param = self.get_parameter("weights").value

        # If user passes an absolute path, use it directly; otherwise load from package share
        if os.path.isabs(weights_param):
            self.weights = weights_param
        else:
            pkg_share = get_package_share_directory("camera_pkg")
            self.weights = os.path.join(pkg_share, "weights", weights_param)  
        self.conf = float(self.get_parameter("conf").value)
        self.iou = float(self.get_parameter("iou").value)
        self.fov_deg = float(self.get_parameter("fov_deg").value)
        self.imgsz = int(self.get_parameter("imgsz").value)

        self.bridge = CvBridge()

        # ONNXRuntime session
        self.session = ort.InferenceSession(self.weights, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name

        self.latest_frame = None
        self.latest_width = None
        self.latest_height = None

        self.sub = self.create_subscription(Image, "/camera/color/image_raw", self.image_cb, 10)

        self.publisher_ = self.create_publisher(Float32MultiArray, "/camera_data", 10)
        self.timer = self.create_timer(0.05, self.publish_target)

        self.get_logger().info(f"{node_name} Ready...")
        self.get_logger().info("Subscribing: /camera/color/image_raw")
        self.get_logger().info(f"Using ONNX: {self.weights}")

    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.latest_frame = frame
        self.latest_height, self.latest_width = frame.shape[:2]

    def _preprocess(self, frame_bgr):
        h0, w0 = frame_bgr.shape[:2]
        img = cv2.resize(frame_bgr, (self.imgsz, self.imgsz), interpolation=cv2.INTER_LINEAR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC -> CHW
        img = np.expand_dims(img, 0)        # -> (1, 3, imgsz, imgsz)
        return img, (w0, h0)

    def _postprocess(self, output, orig_wh):
        w0, h0 = orig_wh

        out = np.squeeze(output[0])  # (5, 8400) or (8400, 5)

        # Ensure shape is (N, 5)
        if out.ndim == 2 and out.shape[0] == 5:
            out = out.T  # -> (8400, 5)

        if out.ndim != 2 or out.shape[1] != 5:
            self.get_logger().warn(f"Unexpected output shape: {out.shape}")
            return []

        # Columns: cx, cy, w, h, score
        cx = out[:, 0]
        cy = out[:, 1]
        bw = out[:, 2]
        bh = out[:, 3]
        scores = out[:, 4]

        # (Optional) If scores look like logits, apply sigmoid
        # If your scores are already 0..1, sigmoid won't hurt much, but can change behavior.
        scores = 1.0 / (1.0 + np.exp(-scores))

        keep = scores >= self.conf
        if not np.any(keep):
            return []

        cx = cx[keep]; cy = cy[keep]; bw = bw[keep]; bh = bh[keep]; scores = scores[keep]

        # Convert cxcywh -> xyxy (in model scale)
        x1 = cx - bw / 2.0
        y1 = cy - bh / 2.0
        x2 = cx + bw / 2.0
        y2 = cy + bh / 2.0

        # Scale from model input to original image size
        sx = w0 / float(self.imgsz)
        sy = h0 / float(self.imgsz)
        x1 *= sx; x2 *= sx
        y1 *= sy; y2 *= sy

        nms_boxes = np.stack([x1, y1, (x2 - x1), (y2 - y1)], axis=1).tolist()
        scores_list = scores.tolist()

        idxs = cv2.dnn.NMSBoxes(nms_boxes, scores_list, self.conf, self.iou)
        if len(idxs) == 0:
            return []

        idxs = np.array(idxs).reshape(-1)
        dets = []
        for i in idxs:
            dets.append((x1[i], y1[i], x2[i], y2[i], float(scores[i]), 0))
        return dets

    def publish_target(self):
        msg = Float32MultiArray()

        if self.latest_frame is None or self.latest_width is None:
            msg.data = [float("inf"), float("inf")]
            self.publisher_.publish(msg)
            return

        width = self.latest_width
        center_x = width / 2.0
        angle_per_pix = self.fov_deg / float(width)

        inp, orig_wh = self._preprocess(self.latest_frame)
        outputs = self.session.run(None, {self.input_name: inp})
        
        # DEBUG CODE GOES HERE
        if not hasattr(self, "_printed_shapes"):
            self.get_logger().info(
                f"ONNX output count={len(outputs)} shapes={[np.array(o).shape for o in outputs]}"
            )
            self._printed_shapes = True

        out = np.squeeze(outputs[0])
        if out.ndim == 2 and out.shape[0] < out.shape[1]:
            out_dbg = out.T
        else:
            out_dbg = out
        self.get_logger().info(
            f"post out shape={out_dbg.shape} min={out_dbg.min():.4f} max={out_dbg.max():.4f}"
        )
        # DEBUG CODE ENDS HERE

        dets = self._postprocess(outputs, orig_wh)

        if len(dets) == 0:
            msg.data = [float("inf"), float("inf")]
            self.publisher_.publish(msg)
            self.get_logger().info("no target")
            return

        # pick best by confidence
        best = max(dets, key=lambda d: d[4])
        x1, y1, x2, y2, score, cls_id = best
        x_center = (x1 + x2) / 2.0

        angle = (center_x - x_center) * angle_per_pix

        msg.data = [0.0, float(angle)]
        self.publisher_.publish(msg)
        self.get_logger().info(f"angle: {angle:.3f} conf: {score:.2f} cls: {cls_id}")

def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
