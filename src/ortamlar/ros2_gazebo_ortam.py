from __future__ import annotations
import time
import math
from typing import Tuple, Dict, Any, Optional
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_srvs.srv import Empty
except Exception:
    rclpy = None  # type: ignore
    Node = object  # type: ignore
    LaserScan = object  # type: ignore
    Odometry = object  # type: ignore
    Twist = object  # type: ignore
    Empty = object  # type: ignore

from ..ros2_entegrasyon.lazer_isleyici import LazerIsleyici
from ..ros2_entegrasyon.odometri_isleyici import OdometriIsleyici
from ..ros2_entegrasyon.hiz_yayinlayici import HizYayinlayici
from ..ros2_entegrasyon.pedsim_isleyici import PedSimIsleyici
from ..ajanlar.davranis_agaci import GenisletilmisDavranisAgaci

class ROS2GazeboOrtam(Node):
    def __init__(
        self,
        scan_topic: str = "/scan",
        odom_topic: str = "/odom",
        cmd_topic: str = "/cmd_vel",
        reset_srv: str = "/reset_world",
        pause_srv: str = "/pause_physics",
        unpause_srv: str = "/unpause_physics",
        pedsim_topic: str = "/pedsim_simulator/simulated_agents",
        sektor_sayisi: int = 10,
        maksimum_menzil: float = 3.5,
        lineer_ileri: float = 0.22,
        lineer_yavas: float = 0.12,
        acisal_donus: float = 1.0,
        hedef_xy: Tuple[float, float] = (8.0, 8.0),
        odul_param: Optional[Dict[str, float]] = None,
        bt: Optional[GenisletilmisDavranisAgaci] = None,
    ) -> None:
        super().__init__("ros2_gazebo_ortam")

        self.lazer = LazerIsleyici(sektor_sayisi=sektor_sayisi, maksimum_menzil=maksimum_menzil)
        self.odom = OdometriIsleyici()
        self.odom.hedef_ayarla(hedef_xy[0], hedef_xy[1])
        self.hiz = HizYayinlayici(lineer_ileri=lineer_ileri, lineer_yavas=lineer_yavas, acisal_donus=acisal_donus)

        self.bt = bt

        self.sub_scan = self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)

        # PedSim (opsiyonel): insan ajan durumları
        self.pedsim = PedSimIsleyici()
        self.pedsim_topic = str(pedsim_topic)
        self.sub_pedsim = None
        if self.pedsim.mesaj_tipi_var_mi():
            try:
                from pedsim_msgs.msg import AgentStates  # type: ignore
                self.sub_pedsim = self.create_subscription(AgentStates, self.pedsim_topic, self._pedsim_cb, 10)
            except Exception:
                # mesaj paketi var gibi görünse bile import başarısız olabilir
                self.sub_pedsim = None

        self.reset_cli = self.create_client(Empty, reset_srv)
        self.pause_cli = self.create_client(Empty, pause_srv)
        self.unpause_cli = self.create_client(Empty, unpause_srv)

        self.odul_param = odul_param or {
            "hedef_ulasildi": 200.0,
            "carpisma": -100.0,
            "zaman_adimi": -0.01,
            "ilerleme_katsayi": 10.0,
            "yakin_ihlal_cezasi": -0.2,
        }

        self.onceki_mesafe: float = 999.0
        self.yakin_ihlal_esik: float = 0.5
        self.carpisma_esik: float = 0.25
        self.hedef_esik: float = 0.35

        self._son_durum: Optional[np.ndarray] = None

    def _pedsim_cb(self, msg) -> None:
        try:
            self.pedsim.isle(msg)
        except Exception:
            pass

    def _scan_cb(self, msg: LaserScan) -> None:
        self.lazer.isle(msg)

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom.isle(msg)

    def _bekle_hazir(self, zaman_asimi: float = 5.0) -> bool:
        bas = time.time()
        while time.time() - bas < zaman_asimi:
            if self.lazer.son_ozet is not None and self.odom.son is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _servis_cagir(self, cli, zaman_asimi: float = 2.0) -> None:
        if not cli.wait_for_service(timeout_sec=zaman_asimi):
            return
        req = Empty.Request()
        fut = cli.call_async(req)
        bas = time.time()
        while rclpy.ok() and not fut.done() and time.time() - bas < zaman_asimi:
            rclpy.spin_once(self, timeout_sec=0.05)

    def sifirla(self) -> Tuple[np.ndarray, Dict[str, Any]]:
        # dur + reset + unpause
        self.pub_cmd.publish(self.hiz.twist_uret(0))
        self._servis_cagir(self.pause_cli)
        self._servis_cagir(self.reset_cli)
        self._servis_cagir(self.unpause_cli)

        if not self._bekle_hazir(5.0):
            raise RuntimeError("Sensör verisi gelmedi. /scan ve /odom kontrol edin.")

        durum = self.durum_al()
        self.onceki_mesafe = float(self.odom.hedefe_mesafe())
        self._son_durum = durum
        bilgi = self.durum_bilgisi()
        return durum, bilgi

    def durum_bilgisi(self) -> Dict[str, Any]:
        # minimum mesafe: PedSim varsa onu tercih et (insan-robot gerçek mesafe), yoksa LIDAR min
        lidar_min = float(self.lazer.minimum_mesafe_al())
        if self.odom.son is not None and self.pedsim.mesaj_tipi_var_mi():
            pedsim_min = float(self.pedsim.en_yakin_mesafe((self.odom.son.x, self.odom.son.y)))
            if math.isfinite(pedsim_min):
                min_mes = pedsim_min
            else:
                min_mes = lidar_min
        else:
            pedsim_min = float("inf")
            min_mes = lidar_min

        ort_hiz, max_hiz = self.pedsim.hiz_ozeti() if self.pedsim.mesaj_tipi_var_mi() else (0.0, 0.0)

        return {
            "minimum_mesafe": float(min_mes),
            "lidar_minimum_mesafe": float(lidar_min),
            "pedsim_minimum_mesafe": float(pedsim_min),
            "insan_sayisi": int(self.pedsim.insan_sayisi()),
            "insan_hiz_ortalama": float(ort_hiz),
            "insan_hiz_maks": float(max_hiz),
            "hedefe_mesafe": self.odom.hedefe_mesafe(),
            "hedefe_aci": self.odom.hedefe_aci(),
            "lineer_hiz": self.odom.son.lineer_hiz if self.odom.son else 0.0,
            "acisal_hiz": self.odom.son.acisal_hiz if self.odom.son else 0.0,
        }

    def durum_al(self) -> np.ndarray:
        lazer = self.lazer.durum_temsili_al()
        odom = self.odom.durum_vektoru_al()
        return np.concatenate([lazer, odom], axis=0).astype(np.float32)

    def adim_at(self, aksiyon: int) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        # aksiyon uygula
        self.pub_cmd.publish(self.hiz.twist_uret(aksiyon))
        # kısa süre fizik ilerlesin
        for _ in range(2):
            rclpy.spin_once(self, timeout_sec=0.05)

        durum = self.durum_al()
        bilgi = self.durum_bilgisi()
        odul = self.odul_hesapla(bilgi)
        bitti = self.bitti_mi(bilgi)

        self._son_durum = durum
        return durum, odul, bitti, bilgi

    def odul_hesapla(self, bilgi: Dict[str, Any]) -> float:
        min_mes = float(bilgi["minimum_mesafe"])
        hedef_mes = float(bilgi["hedefe_mesafe"])

        # ilerleme
        ilerleme = (self.onceki_mesafe - hedef_mes) * float(self.odul_param["ilerleme_katsayi"])
        self.onceki_mesafe = hedef_mes

        odul = float(self.odul_param["zaman_adimi"]) + ilerleme

        # yakın ihlal cezası
        if min_mes < self.yakin_ihlal_esik:
            odul += float(self.odul_param["yakin_ihlal_cezasi"])

        # çarpışma / hedef
        if min_mes < self.carpisma_esik:
            odul += float(self.odul_param["carpisma"])
        if hedef_mes < self.hedef_esik:
            odul += float(self.odul_param["hedef_ulasildi"])

        return float(odul)

    def bitti_mi(self, bilgi: Dict[str, Any]) -> bool:
        min_mes = float(bilgi["minimum_mesafe"])
        hedef_mes = float(bilgi["hedefe_mesafe"])
        if min_mes < self.carpisma_esik:
            return True
        if hedef_mes < self.hedef_esik:
            return True
        return False
