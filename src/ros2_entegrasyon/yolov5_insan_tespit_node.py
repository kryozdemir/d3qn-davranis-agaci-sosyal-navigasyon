from __future__ import annotations
import os
import time

import numpy as np
import torch

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D


class Yolov5InsanTespitDugumu(Node):
    def __init__(self) -> None:
        super().__init__("yolov5_insan_tespit")

        self.declare_parameter("gorsel_topic", "/camera/image_raw")
        self.declare_parameter("yolov5_repo_yolu", os.path.expanduser("~/ros2_ws/src/yolov5"))
        self.declare_parameter("agirlik_yolu", "")
        self.declare_parameter("guven_esigi", 0.35)
        self.declare_parameter("iou_esigi", 0.45)
        self.declare_parameter("goruntu_boyutu", 640)
        self.declare_parameter("cihaz", "auto")
        self.declare_parameter("insan_var_esigi", 1)

        self.gorsel_topic = self.get_parameter("gorsel_topic").get_parameter_value().string_value
        self.yolov5_repo_yolu = self.get_parameter("yolov5_repo_yolu").get_parameter_value().string_value
        self.agirlik_yolu = self.get_parameter("agirlik_yolu").get_parameter_value().string_value
        self.guven_esigi = float(self.get_parameter("guven_esigi").value)
        self.iou_esigi = float(self.get_parameter("iou_esigi").value)
        self.goruntu_boyutu = int(self.get_parameter("goruntu_boyutu").value)
        self.cihaz_param = self.get_parameter("cihaz").get_parameter_value().string_value
        self.insan_var_esigi = int(self.get_parameter("insan_var_esigi").value)

        self.bridge = CvBridge()
        self.tespit_yayinci = self.create_publisher(Detection2DArray, "/insan_tespitleri", 10)
        self.insan_var_yayinci = self.create_publisher(Bool, "/insan_var", 10)
        self.abone = self.create_subscription(Image, self.gorsel_topic, self._gorsel_cb, 10)

        self.cihaz = self._cihaz_sec(self.cihaz_param)
        self.model = self._model_yukle()

        self.son_log_saniye = 0

    def _cihaz_sec(self, cihaz_param: str) -> str:
        if cihaz_param == "cpu":
            return "cpu"
        if cihaz_param == "cuda":
            return "cuda" if torch.cuda.is_available() else "cpu"
        return "cuda" if torch.cuda.is_available() else "cpu"

    def _model_yukle(self):
        if not os.path.isdir(self.yolov5_repo_yolu) or not os.path.isfile(os.path.join(self.yolov5_repo_yolu, "hubconf.py")):
            raise RuntimeError("yolov5_repo_yolu gecersiz: " + str(self.yolov5_repo_yolu))

        if self.agirlik_yolu and os.path.isfile(self.agirlik_yolu):
            model = torch.hub.load(self.yolov5_repo_yolu, "custom", path=self.agirlik_yolu, source="local")
        else:
            model = torch.hub.load(self.yolov5_repo_yolu, "yolov5s", pretrained=True, source="local")

        model.to(self.cihaz)
        model.conf = self.guven_esigi
        model.iou = self.iou_esigi
        model.classes = [0]
        model.eval()
        return model

    def _gorsel_cb(self, msg: Image) -> None:
        try:
            goruntu = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return

        t0 = time.time()
        sonuc = self.model(goruntu, size=self.goruntu_boyutu)
        sure_ms = (time.time() - t0) * 1000.0

        tespitler = sonuc.xyxy[0].detach().cpu().numpy() if hasattr(sonuc, "xyxy") and len(sonuc.xyxy) > 0 else np.zeros((0, 6), dtype=np.float32)

        dizi = Detection2DArray()
        dizi.header = msg.header

        kisi_sayisi = 0
        for satir in tespitler:
            x1, y1, x2, y2, guven, sinif_id = satir.tolist()
            if int(sinif_id) != 0:
                continue
            kisi_sayisi += 1
            w = max(0.0, x2 - x1)
            h = max(0.0, y2 - y1)
            cx = x1 + w / 2.0
            cy = y1 + h / 2.0

            tespit = Detection2D()
            tespit.header = dizi.header

            bbox = BoundingBox2D()
            bbox.center.x = float(cx)
            bbox.center.y = float(cy)
            bbox.size_x = float(w)
            bbox.size_y = float(h)
            tespit.bbox = bbox

            hip = ObjectHypothesisWithPose()
            hip.hypothesis.class_id = "person"
            hip.hypothesis.score = float(guven)
            tespit.results.append(hip)

            dizi.detections.append(tespit)

        self.tespit_yayinci.publish(dizi)

        insan_var = Bool()
        insan_var.data = bool(kisi_sayisi >= self.insan_var_esigi)
        self.insan_var_yayinci.publish(insan_var)

        simdi = int(self.get_clock().now().nanoseconds / 1e9)
        if simdi != self.son_log_saniye and simdi % 5 == 0:
            self.son_log_saniye = simdi
            self.get_logger().info(f"kisi={kisi_sayisi} sure_ms={sure_ms:.1f} cihaz={self.cihaz}")


def main(args=None) -> None:
    rclpy.init(args=args)
    dugum = Yolov5InsanTespitDugumu()
    try:
        rclpy.spin(dugum)
    except KeyboardInterrupt:
        pass
    finally:
        dugum.destroy_node()
        rclpy.shutdown()
