from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Tuple
import math
import numpy as np

try:
    from nav_msgs.msg import Odometry
except Exception:
    Odometry = object  # type: ignore

@dataclass
class OdomOzeti:
    x: float
    y: float
    yaw: float
    lineer_hiz: float
    acisal_hiz: float

class OdometriIsleyici:
    def __init__(self) -> None:
        self.son: Optional[OdomOzeti] = None
        self.hedef_x: float = 0.0
        self.hedef_y: float = 0.0

    def hedef_ayarla(self, x: float, y: float) -> None:
        self.hedef_x = float(x)
        self.hedef_y = float(y)

    def quaternion_euler(self, x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
        # roll, pitch, yaw
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw

    def isle(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = self.quaternion_euler(q.x, q.y, q.z, q.w)
        lv = msg.twist.twist.linear.x
        av = msg.twist.twist.angular.z
        self.son = OdomOzeti(x=float(p.x), y=float(p.y), yaw=float(yaw), lineer_hiz=float(lv), acisal_hiz=float(av))

    def hedefe_mesafe(self) -> float:
        if not self.son:
            return 999.0
        dx = self.hedef_x - self.son.x
        dy = self.hedef_y - self.son.y
        return float(math.hypot(dx, dy))

    def hedefe_aci(self) -> float:
        if not self.son:
            return 0.0
        dx = self.hedef_x - self.son.x
        dy = self.hedef_y - self.son.y
        hedef_yon = math.atan2(dy, dx)
        fark = hedef_yon - self.son.yaw
        # [-pi, pi]
        while fark > math.pi:
            fark -= 2 * math.pi
        while fark < -math.pi:
            fark += 2 * math.pi
        return float(fark)

    def durum_vektoru_al(self) -> np.ndarray:
        # 4 boyut: hedef mesafe (normalize), hedef açı (normalize), lineer hız, açısal hız
        mes = self.hedefe_mesafe()
        aci = self.hedefe_aci()
        lineer = self.son.lineer_hiz if self.son else 0.0
        acisal = self.son.acisal_hiz if self.son else 0.0
        mes_n = min(mes / 15.0, 1.0)
        aci_n = aci / math.pi  # [-1,1]
        return np.array([mes_n, aci_n, lineer, acisal], dtype=np.float32)
