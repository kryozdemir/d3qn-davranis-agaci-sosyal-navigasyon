#!/usr/bin/env python3
from typing import Dict, Any, Optional, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

from ajanlar.d3qn_ajan import D3QNAjan
from ajanlar.davranis_agaci import GenisletilmisDavranisAgaci
from ortamlar.ros2_gazebo_ortam import ROS2GazeboOrtam


class AnaNode(Node):
    def __init__(self):
        super().__init__('ana_node')

        self.declare_parameter('parametreler_yaml', '')
        self.declare_parameter('senaryo_yaml', '')
        self.declare_parameter('bt_aktif', True)
        self.declare_parameter('model_yolu', '')
        self.declare_parameter('egitim_modu', False)
        self.declare_parameter('insan_var_topic', '/insan_var')

        self.parametreler_yaml = self.get_parameter('parametreler_yaml').get_parameter_value().string_value
        self.senaryo_yaml = self.get_parameter('senaryo_yaml').get_parameter_value().string_value
        self.bt_aktif = bool(self.get_parameter('bt_aktif').value)
        self.model_yolu = self.get_parameter('model_yolu').get_parameter_value().string_value
        self.egitim_modu = bool(self.get_parameter('egitim_modu').value)
        self.insan_var_topic = self.get_parameter('insan_var_topic').get_parameter_value().string_value

        self.insan_var = False
        self.create_subscription(Bool, self.insan_var_topic, self._insan_var_cb, 10)

        self.kara_tahta: Dict[str, Any] = {}

        self.ortam = ROS2GazeboOrtam(parametreler_yaml=self.parametreler_yaml, senaryo_yaml=self.senaryo_yaml)
        self.ajan = D3QNAjan(parametreler_yaml=self.parametreler_yaml)

        if self.model_yolu:
            try:
                self.ajan.yukle(self.model_yolu)
            except Exception:
                pass

        self.davranis_agaci = GenisletilmisDavranisAgaci(parametreler_yaml=self.parametreler_yaml)
        self.zamanlayici = self.create_timer(0.1, self._dongu)

        self.son_durum: Optional[Any] = None

    def _insan_var_cb(self, msg: Bool):
        self.insan_var = bool(msg.data)

    def _kara_tahta_guncelle(self, durum, bilgi: Dict[str, Any]):
        self.kara_tahta['insan_var'] = self.insan_var
        self.kara_tahta['pedsim_minimum_mesafe'] = float(bilgi.get('pedsim_minimum_mesafe', 999.0))
        self.kara_tahta['lidar_minimum_mesafe'] = float(bilgi.get('lidar_minimum_mesafe', 999.0))
        self.kara_tahta['minimum_mesafe'] = float(bilgi.get('minimum_mesafe', 999.0))
        self.kara_tahta['hedef_mesafe'] = float(bilgi.get('hedef_mesafe', 0.0))
        self.kara_tahta['hedef_aci'] = float(bilgi.get('hedef_aci', 0.0))
        self.kara_tahta['robot_dogrusal_hiz'] = float(bilgi.get('robot_dogrusal_hiz', 0.0))
        self.kara_tahta['robot_acisal_hiz'] = float(bilgi.get('robot_acisal_hiz', 0.0))

    def _aksiyon_sec(self, durum_vektoru) -> int:
        if self.egitim_modu:
            return self.ajan.aksiyon_sec(durum_vektoru, kesif=True)
        return self.ajan.aksiyon_sec(durum_vektoru, kesif=False)

    def _aksiyon_kisitla(self, aksiyon: int) -> Tuple[int, Dict[str, Any]]:
        if not self.bt_aktif:
            return aksiyon, {}

        self.davranis_agaci.guncelle(self.kara_tahta)
        kisit = self.davranis_agaci.aksiyon_kisitlama_al()
        izinli = kisit.get('izinli_aksiyonlar', None)

        if isinstance(izinli, (list, tuple)) and len(izinli) > 0:
            if aksiyon not in izinli:
                aksiyon = int(izinli[0])

        return aksiyon, kisit

    def _dongu(self):
        try:
            durum = self.ortam.durum_al()
            bilgi = self.ortam.durum_bilgisi()
        except Exception:
            return

        self._kara_tahta_guncelle(durum, bilgi)

        try:
            aksiyon = self._aksiyon_sec(durum)
        except Exception:
            return

        aksiyon, kisit = self._aksiyon_kisitla(aksiyon)

        try:
            self.ortam.adim_at(aksiyon, hiz_kisitlari=kisit)
        except Exception:
            return


def main(args=None):
    rclpy.init(args=args)
    node = AnaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
