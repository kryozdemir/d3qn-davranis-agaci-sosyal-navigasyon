from __future__ import annotations
import os
import time
from typing import Optional, Dict, Any

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from .lazer_isleyici import LazerIsleyici
from .odometri_isleyici import OdometriIsleyici
from .hiz_yayinlayici import HizYayinlayici
from .pedsim_isleyici import PedSimIsleyici
from ..ajanlar.d3qn_ajan import D3QNAjan
from ..ajanlar.davranis_agaci import GenisletilmisDavranisAgaci


class AnaDugum(Node):
    def __init__(self) -> None:
        super().__init__("d3qn_sosyal_navigasyon")

        param_yol = self.declare_parameter("parametreler", "").get_parameter_value().string_value
        senaryo_yol = self.declare_parameter("senaryo", "").get_parameter_value().string_value
        robot_param_yol = self.declare_parameter("robot_parametreler", "").get_parameter_value().string_value
        model_yol = self.declare_parameter("model_yolu", "").get_parameter_value().string_value
        bt_aktif = self.declare_parameter("bt_aktif", True).get_parameter_value().bool_value
        pedsim_topic = self.declare_parameter("pedsim_topic", "/pedsim_simulator/simulated_agents").get_parameter_value().string_value

        if not param_yol or not os.path.exists(param_yol):
            raise RuntimeError("parametreler dosyasi bulunamadi: " + str(param_yol))
        if not senaryo_yol or not os.path.exists(senaryo_yol):
            raise RuntimeError("senaryo dosyasi bulunamadi: " + str(senaryo_yol))
        if not robot_param_yol:
            robot_param_yol = os.path.join(os.path.dirname(param_yol), "robot_parametreler.yaml")
        if not os.path.exists(robot_param_yol):
            raise RuntimeError("robot_parametreler dosyasi bulunamadi: " + str(robot_param_yol))

        with open(param_yol, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        with open(senaryo_yol, "r", encoding="utf-8") as f:
            sc = yaml.safe_load(f)
        with open(robot_param_yol, "r", encoding="utf-8") as f:
            rcfg = yaml.safe_load(f)

        y = cfg["yamlag"]
        d = cfg["davranis_agaci"]
        r = rcfg["robot"]

        hedef = sc["yamlsenaryo"]["navigasyon"]["hedef"]
        hedef_x, hedef_y = float(hedef["x"]), float(hedef["y"])

        self.lazer = LazerIsleyici(sektor_sayisi=int(r["lazer"]["sektor_sayisi"]), maksimum_menzil=float(r["lazer"]["maksimum_menzil"]))
        self.odom = OdometriIsleyici()
        self.odom.hedef_ayarla(hedef_x, hedef_y)
        self.hiz = HizYayinlayici(
            lineer_ileri=float(r["lineer_hiz"]["ileri"]),
            lineer_yavas=float(r["lineer_hiz"]["yavas"]),
            acisal_donus=float(r["acisal_hiz"]["donus"]),
        )

        self.bt_aktif = bool(bt_aktif)
        self.bt = GenisletilmisDavranisAgaci(
            carpisma_esik=float(d.get("carpisma_esik", 0.25)),
            hedef_yaklasma_esik=float(d.get("hedef_yaklasma_esik", 1.0)),
            yakin_mesafe_esik=float(d.get("yakin_mesafe_esik", 0.5)),
        )

        self.pedsim = PedSimIsleyici()
        self.insan_var: bool = False

        self.durum_boyut = int(y["durum_boyut"])
        self.aksiyon_boyut = int(y["aksiyon_boyut"])
        gizli = int(y["gizli_boyut"])

        self.ajan = D3QNAjan(
            durum_boyut=self.durum_boyut,
            aksiyon_boyut=self.aksiyon_boyut,
            gizli_boyut=gizli,
            bellek_boyutu=1,
            parti_boyutu=1,
            ogrenme_orani=1e-4,
            gamma=0.99,
            epsilon_baslangic=0.0,
            epsilon_bitis=0.0,
            epsilon_azalma=1.0,
        )
        if model_yol:
            self.ajan.yukle(model_yol)

        scan_topic = r.get("scan_topic", "/scan")
        odom_topic = r.get("odom_topic", "/odom")
        cmd_topic = r.get("cmd_vel_topic", "/cmd_vel")

        self.sub_scan = self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.sub_pedsim = self.create_subscription(type(self.pedsim.ros_mesaj_tipi()), pedsim_topic, self._pedsim_cb, 10) if self.pedsim.ros_mesaj_tipi() is not None else None
        self.sub_insan_var = self.create_subscription(Bool, "/insan_var", self._insan_var_cb, 10)

        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)

        self.son_adim_zamani = 0.0
        dongu_suresi = self.declare_parameter("dongu_suresi", 0.10).get_parameter_value().double_value
        self.create_timer(float(dongu_suresi), self._kontrol_dongusu)

    def _scan_cb(self, msg: LaserScan) -> None:
        self.lazer.isle(msg)

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom.isle(msg)

    def _pedsim_cb(self, msg) -> None:
        try:
            self.pedsim.isle(msg)
        except Exception:
            return

    def _insan_var_cb(self, msg: Bool) -> None:
        self.insan_var = bool(msg.data)

    def _durum_olustur(self) -> np.ndarray:
        lazer = self.lazer.durum_temsili_al()
        odom = self.odom.durum_vektoru_al()
        durum = np.concatenate([lazer, odom], axis=0).astype(np.float32)
        if durum.shape[0] != self.durum_boyut:
            if durum.shape[0] > self.durum_boyut:
                durum = durum[: self.durum_boyut]
            else:
                pad = np.zeros((self.durum_boyut - durum.shape[0],), dtype=np.float32)
                durum = np.concatenate([durum, pad], axis=0)
        return durum

    def _durum_bilgisi(self) -> Dict[str, Any]:
        rx = self.odom.son.x if self.odom.son else 0.0
        ry = self.odom.son.y if self.odom.son else 0.0
        pedsim_min = self.pedsim.minimum_mesafe(rx, ry) if self.pedsim.var_mi() else float("inf")
        lidar_min = self.lazer.minimum_mesafe_al()
        min_mesafe = float(min(pedsim_min, lidar_min))
        bilgi = {
            "minimum_mesafe": min_mesafe,
            "hedefe_mesafe": self.odom.hedefe_mesafe(),
            "hedefe_aci": self.odom.hedefe_aci(),
            "lineer_hiz": float(self.odom.son.lineer_hiz) if self.odom.son else 0.0,
            "acisal_hiz": float(self.odom.son.acisal_hiz) if self.odom.son else 0.0,
            "insan_var": bool(self.insan_var),
        }
        return bilgi

    def _aksiyon_sec(self, durum: np.ndarray, kisitlar: Dict[int, float]) -> int:
        aksiyon = int(self.ajan.aksiyon_sec(durum, kesif_izinli=False))
        if not kisitlar:
            return aksiyon
        if aksiyon not in kisitlar:
            return aksiyon
        cezali = set(kisitlar.keys())
        adaylar = [a for a in range(self.aksiyon_boyut) if a not in cezali]
        if not adaylar:
            return aksiyon
        return int(adaylar[0])

    def _kontrol_dongusu(self) -> None:
        simdi = time.time()
        if simdi - self.son_adim_zamani < 0.01:
            return
        self.son_adim_zamani = simdi

        durum = self._durum_olustur()
        bilgi = self._durum_bilgisi()

        if self.bt_aktif:
            self.bt.guncelle(bilgi)
            kisit = self.bt.aksiyon_kisitlama_al()
        else:
            kisit = {}

        aksiyon = self._aksiyon_sec(durum, kisit)
        twist = self.hiz.twist_uret(aksiyon)
        self.pub_cmd.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AnaDugum()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
