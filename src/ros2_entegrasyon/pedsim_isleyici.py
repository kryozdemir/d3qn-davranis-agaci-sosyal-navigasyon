from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple
import math

try:
    # PedSim ROS2 mesaj paketi
    from pedsim_msgs.msg import AgentStates  # type: ignore
except Exception:
    AgentStates = None  # type: ignore


@dataclass
class InsanDurumu:
    ajan_id: int
    x: float
    y: float
    vx: float
    vy: float


class PedSimIsleyici:
    """PedSim'den insan ajan durumlarını toplayan hafif işleyici.

    Bu sınıf, PedSim'in yayınladığı AgentStates mesajını parse eder ve
    robot-ajan mesafesi gibi metriklerin hesaplanmasına yardımcı olur.

    Notlar:
    - PedSim mesaj paketi sistemde yoksa AgentStates None olur ve sınıf pasif çalışır.
    - Topic adı parametre ile değiştirilebilir.
    """

    def __init__(self) -> None:
        self.aktif: bool = AgentStates is not None
        self.insanlar: List[InsanDurumu] = []
        self.son_mesaj_zamani: Optional[float] = None

    def mesaj_tipi_var_mi(self) -> bool:
        return bool(self.aktif)

    def isle(self, msg: AgentStates) -> None:
        # AgentStates.msg içinde genelde "agent_states" alanı bulunur.
        insanlar: List[InsanDurumu] = []
        try:
            liste = getattr(msg, "agent_states")
        except Exception:
            liste = []

        for a in list(liste):
            try:
                ajan_id = int(getattr(a, "id", 0))
                px = float(a.pose.position.x)
                py = float(a.pose.position.y)

                # twist her zaman dolu olmayabilir
                vx = float(getattr(a.twist.linear, "x", 0.0))
                vy = float(getattr(a.twist.linear, "y", 0.0))
            except Exception:
                continue
            insanlar.append(InsanDurumu(ajan_id=ajan_id, x=px, y=py, vx=vx, vy=vy))

        self.insanlar = insanlar

    def insan_sayisi(self) -> int:
        return int(len(self.insanlar))

    def en_yakin_mesafe(self, robot_xy: Tuple[float, float]) -> float:
        """Robot ile en yakın insan arasındaki öklidyen mesafeyi döndürür."""
        if not self.insanlar:
            return float("inf")
        rx, ry = float(robot_xy[0]), float(robot_xy[1])
        en_kucuk = float("inf")
        for i in self.insanlar:
            d = math.hypot(i.x - rx, i.y - ry)
            if d < en_kucuk:
                en_kucuk = d
        return float(en_kucuk)

    def hiz_ozeti(self) -> Tuple[float, float]:
        """(ortalama_hiz, maksimum_hiz) döndürür."""
        if not self.insanlar:
            return (0.0, 0.0)
        hizlar = [math.hypot(i.vx, i.vy) for i in self.insanlar]
        ort = float(sum(hizlar) / max(1, len(hizlar)))
        mx = float(max(hizlar))
        return (ort, mx)
