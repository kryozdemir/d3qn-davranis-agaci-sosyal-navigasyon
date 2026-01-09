from __future__ import annotations
from typing import Dict, Tuple
import time

try:
    from geometry_msgs.msg import Twist
except Exception:
    Twist = object  # type: ignore

class HizYayinlayici:
    def __init__(self, lineer_ileri: float = 0.22, lineer_yavas: float = 0.12, acisal_donus: float = 1.0) -> None:
        self.lineer_ileri = float(lineer_ileri)
        self.lineer_yavas = float(lineer_yavas)
        self.acisal_donus = float(acisal_donus)
        self.aksiyon_haritasi = self.aksiyon_haritasi_olustur()

    def aksiyon_haritasi_olustur(self) -> Dict[int, Tuple[float, float]]:
        # (lineer, acisal)
        return {
            0: (self.lineer_ileri, 0.0),
            1: (self.lineer_yavas, +self.acisal_donus),
            2: (self.lineer_yavas, -self.acisal_donus),
            3: (0.0, +self.acisal_donus),
            4: (0.0, -self.acisal_donus),
        }

    def twist_uret(self, aksiyon: int) -> Twist:
        msg = Twist()
        lineer, acisal = self.aksiyon_haritasi.get(int(aksiyon), (0.0, 0.0))
        msg.linear.x = float(lineer)
        msg.angular.z = float(acisal)
        return msg
