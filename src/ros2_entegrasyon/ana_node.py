from __future__ import annotations
import os
import time
from typing import Optional, Dict, Any

import numpy as np
import yaml

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
except Exception as e:
    rclpy = None  # type: ignore
    Node = object  # type: ignore
    LaserScan = object  # type: ignore
    Odometry = object  # type: ignore
    Twist = object  # type: ignore

from .lazer_isleyici import LazerIsleyici
from .odometri_isleyici import OdometriIsleyici
from .hiz_yayinlayici import HizYayinlayici
from .pedsim_isleyici import PedSimIsleyici
from ..ajanlar.d3qn_ajan import D3QNAjan
from ..ajanlar.davranis_agaci import GenisletilmisDavranisAgaci

class AnaDugum(Node):
    def __init__(self) -> None:
        super().__init__("d3qn_sosyal_navigasyon")

        # parametreler dosyadan okunur
        param_yol = self.declare_parameter("parametreler", "").get_parameter_value().string_value
        senaryo_yol = self.declare_parameter("senaryo", "").get_parameter_value().string_value
        bt_aktif = self.declare_parameter("bt_aktif", True).get_parameter_value().bool_value
        pedsim_topic = self.declare_parameter("pedsim_topic", "/pedsim_simulator/simulated_agents").get_parameter_value().string_value

        if not param_yol or not os.path.exists(param_yol):
            raise RuntimeError("parametreler dosyasi bulunamadi: " + str(param_yol))
        if not senaryo_yol or not os.path.exists(senaryo_yol):
            raise RuntimeError("senaryo dosyasi bulunamadi: " + str(senaryo_yol))

        with open(param_yol, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        with open(senaryo_yol, "r", encoding="utf-8") as f:
            sc = yaml.safe_load(f)

        y = cfg["yamlag"]
        e = cfg["egitim"]
        d = cfg["davranis_agaci"]
        r = yaml.safe_load(open(os.path.join(os.path.dirname(param_yol), "robot_parametreler.yaml"), "r", encoding="utf-8"))["robot"]

        hedef = sc["yamlsenaryo"]["navigasyon"]["hedef"]
        hedef_x, hedef_y = float(hedef["x"]), float(hedef["y"])

        self.lazer = LazerIsleyici(sektor_sayisi=int(r["lazer"]["sektor_sayisi"]), maksimum_menzil=float(r["lazer"]["maksimum_menzil"]))
        self.odom = OdometriIsleyici()
        self.odom.hedef_ayarla(hedef_x, hedef_y)
        self.hiz = HizYayinlayici(lineer_ileri=float(r["lineer_hiz"]["ileri"]),
                                  lineer_yavas=float(r["lineer_hiz"]["yavas"]),
                                  acisal_donus=float(r["acisal_hiz"]["donus"]))

        self.ajan = D3QNAjan(
            durum_boyut=int(y["durum_boyut"]),
            aksiyon_boyut=int(y["aksiyon_boyut"]),
            gizli_boyut=int(y["gizli_boyut"]),
            ogrenme_orani=float(e["ogrenme_orani"]),
            gamma=float(e["gamma"]),
            epsilon_baslangic=float(e["epsilon_baslangic"]),
            epsilon_bitis=float(e["epsilon_bitis"]),
            epsilon_azalma=float(e["epsilon_azalma"]),
            bellek_boyutu=int(e["bellek_boyutu"]),
            hedef_ag_guncelleme_adimi=int(e["hedef_ag_guncelleme_adimi"]),
            hedef_ag_yumusak_guncelleme=float(e["hedef_ag_yumusak_guncelleme"]),
        )

        self.bt_aktif = bool(bt_aktif and d.get("aktif", True))
        self.bt = GenisletilmisDavranisAgaci(
            carpisma_esik=float(d["carpisma_esik"]),
            hedef_yaklasma_esik=float(d["hedef_yaklasma_esik"]),
            yakin_mesafe_esik=float(d["yakin_mesafe_esik"]),
        )

        scan_topic = r["scan_topic"]
        odom_topic = r["odom_topic"]
        cmd_topic = r["cmd_vel_topic"]

        self.sub_scan = self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)

        # PedSim (opsiyonel): BT için daha doğru yakınlık hesabı
        self.pedsim = PedSimIsleyici()
        self.sub_pedsim = None
        if self.pedsim.mesaj_tipi_var_mi():
            try:
                from pedsim_msgs.msg import AgentStates  # type: ignore
                self.sub_pedsim = self.create_subscription(AgentStates, str(pedsim_topic), self._pedsim_cb, 10)
                self.get_logger().info("PedSim topic aboneliği etkin: %s" % str(pedsim_topic))
            except Exception:
                self.sub_pedsim = None

        self.son_aksiyon: int = 0
        self.zaman = self.get_clock()

        # kontrol döngüsü
        self.timer = self.create_timer(0.1, self._adim)

        self.get_logger().info("Ana dugum basladi. Senaryo: %s" % sc["yamlsenaryo"]["isim"])

    def _scan_cb(self, msg: LaserScan) -> None:
        self.lazer.isle(msg)

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom.isle(msg)

    def _pedsim_cb(self, msg) -> None:
        try:
            self.pedsim.isle(msg)
        except Exception:
            pass

    def _durum_bilgisi(self) -> Dict[str, Any]:
        lidar_min = float(self.lazer.minimum_mesafe_al())
        if self.odom.son is not None and self.pedsim.mesaj_tipi_var_mi():
            pedsim_min = float(self.pedsim.en_yakin_mesafe((self.odom.son.x, self.odom.son.y)))
            min_mes = pedsim_min if np.isfinite(pedsim_min) else lidar_min
        else:
            pedsim_min = float("inf")
            min_mes = lidar_min

        return {
            "minimum_mesafe": float(min_mes),
            "lidar_minimum_mesafe": float(lidar_min),
            "pedsim_minimum_mesafe": float(pedsim_min),
            "insan_sayisi": int(self.pedsim.insan_sayisi()),
            "hedefe_mesafe": self.odom.hedefe_mesafe(),
            "hedefe_aci": self.odom.hedefe_aci(),
            "lineer_hiz": self.odom.son.lineer_hiz if self.odom.son else 0.0,
            "acisal_hiz": self.odom.son.acisal_hiz if self.odom.son else 0.0,
        }

    def _durum_vektoru(self) -> np.ndarray:
        lazer = self.lazer.durum_temsili_al()  # 10
        odom = self.odom.durum_vektoru_al()    # 4
        return np.concatenate([lazer, odom], axis=0).astype(np.float32)

    def _adim(self) -> None:
        # sensörler hazır değilse dur
        if self.lazer.son_ozet is None or self.odom.son is None:
            return

        # BT güncelle
        if self.bt_aktif:
            self.bt.guncelle(self._durum_bilgisi())
            kesif_izinli = self.bt.kesif_izinli_mi()
            kisitlar = self.bt.aksiyon_kisitlama_al()
        else:
            kesif_izinli = True
            kisitlar = {}

        durum = self._durum_vektoru()
        aksiyon = self.ajan.aksiyon_sec(durum, kesif_izinli=kesif_izinli)

        # yumuşak kısıt: bazı aksiyonlar seçildiyse güvenli alternatife düş
        if aksiyon in kisitlar and kisitlar[aksiyon] >= 0.9:
            aksiyon = 3 if aksiyon in (0,1,2) else 0

        twist = self.hiz.twist_uret(aksiyon)
        self.pub_cmd.publish(twist)
        self.son_aksiyon = aksiyon

def main() -> None:
    if rclpy is None:
        raise RuntimeError("ROS2 (rclpy) bulunamadi. Bu dugum ROS2 ortaminda calistirilmalidir.")
    rclpy.init()
    node = AnaDugum()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
