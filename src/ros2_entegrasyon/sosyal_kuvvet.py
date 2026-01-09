from __future__ import annotations
from dataclasses import dataclass
from typing import List, Tuple
import math

@dataclass
class AjanDurumu:
    x: float
    y: float
    vx: float
    vy: float
    kitle: float = 60.0
    alpha: float = 1.0     # sosyal tutum katsayısı
    kisisel_yaricap: float = 0.6  # r_i (m)

class SosyalKuvvetModeli:
    # Bu sınıf, yaya dinamiğini simülasyon içinde hesaplamak isteyenler için
    # bağımsız (ROS2'den ayrı) bir hesap modülü sağlar.
    def __init__(self, A: float = 2.1, B: float = 0.3) -> None:
        self.A = float(A)
        self.B = float(B)

    def robot_insan_kuvveti(self, insan: AjanDurumu, robot_xy: Tuple[float, float]) -> Tuple[float, float]:
        # Denklem (2): F_i^robot = alpha_i * A * exp((r_i - d_ir)/B) * n_ir
        rx, ry = robot_xy
        dx = insan.x - rx
        dy = insan.y - ry
        d = math.hypot(dx, dy)
        if d < 1e-6:
            # çakışma durumunda rastgele yön
            return (0.0, 0.0)

        n_x = dx / d
        n_y = dy / d
        kuvvet = insan.alpha * self.A * math.exp((insan.kisisel_yaricap - d) / self.B)
        return (kuvvet * n_x, kuvvet * n_y)

    def toplam_robot_kuvveti(self, insanlar: List[AjanDurumu], robot_xy: Tuple[float, float]) -> List[Tuple[float, float]]:
        return [self.robot_insan_kuvveti(i, robot_xy) for i in insanlar]
