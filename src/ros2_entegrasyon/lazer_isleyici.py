from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, List
import numpy as np

try:
    from sensor_msgs.msg import LaserScan
except Exception:  # ROS2 yoksa import hatası alınır
    LaserScan = object  # type: ignore

@dataclass
class LazerOzeti:
    sektor_degerleri: np.ndarray
    minimum_mesafe: float

class LazerIsleyici:
    def __init__(self, sektor_sayisi: int = 10, maksimum_menzil: float = 3.5) -> None:
        self.sektor_sayisi = int(sektor_sayisi)
        self.maksimum_menzil = float(maksimum_menzil)
        self.son_ozet: Optional[LazerOzeti] = None

    def isle(self, msg: LaserScan) -> None:
        menziller = np.array(list(msg.ranges), dtype=np.float32)
        menziller = np.nan_to_num(menziller, nan=self.maksimum_menzil, posinf=self.maksimum_menzil, neginf=self.maksimum_menzil)
        menziller = np.clip(menziller, 0.0, self.maksimum_menzil)

        n = len(menziller)
        if n == 0:
            sektor = np.ones((self.sektor_sayisi,), dtype=np.float32) * self.maksimum_menzil
            self.son_ozet = LazerOzeti(sektor_degerleri=sektor, minimum_mesafe=float(self.maksimum_menzil))
            return

        parca = max(1, n // self.sektor_sayisi)
        sektorler: List[float] = []
        for i in range(self.sektor_sayisi):
            bas = i * parca
            bit = (i + 1) * parca if i < self.sektor_sayisi - 1 else n
            sektorler.append(float(np.min(menziller[bas:bit])))

        sektor = np.array(sektorler, dtype=np.float32)
        self.son_ozet = LazerOzeti(sektor_degerleri=sektor, minimum_mesafe=float(np.min(menziller)))

    def minimum_mesafe_al(self) -> float:
        return float(self.son_ozet.minimum_mesafe) if self.son_ozet else float(self.maksimum_menzil)

    def durum_temsili_al(self) -> np.ndarray:
        if not self.son_ozet:
            return np.ones((self.sektor_sayisi,), dtype=np.float32)
        # normalize [0,1] (1 uzak, 0 yakın)
        return np.clip(self.son_ozet.sektor_degerleri / self.maksimum_menzil, 0.0, 1.0).astype(np.float32)
