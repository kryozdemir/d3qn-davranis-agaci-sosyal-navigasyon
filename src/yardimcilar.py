from __future__ import annotations
import os, json, csv, time, math
from dataclasses import dataclass, asdict
from typing import Dict, Any, List, Optional

def dizin_olustur(yol: str) -> None:
    os.makedirs(yol, exist_ok=True)

def zaman_damgasi() -> str:
    return time.strftime("%Y%m%d_%H%M%S")

@dataclass
class EpisodeMetrikleri:
    episode: int
    basarili: bool
    carpisma: bool
    toplam_odul: float
    adim_sayisi: int
    hedefe_ulasma_suresi: Optional[float]
    yakin_ihlal_adim: int
    toplam_adim: int
    gercek_yol_uzunlugu: float
    optimal_yol_uzunlugu: float
    sim_hatasi: bool = False

    def yakin_ihlal_orani_yuzde(self) -> float:
        if self.toplam_adim <= 0:
            return 0.0
        return (self.yakin_ihlal_adim / self.toplam_adim) * 100.0

    def yol_verimliligi(self) -> float:
        if self.gercek_yol_uzunlugu <= 1e-9:
            return 0.0
        return self.optimal_yol_uzunlugu / self.gercek_yol_uzunlugu

class MetrikToplayici:
    def __init__(self) -> None:
        self.episode_listesi: List[EpisodeMetrikleri] = []

    def ekle(self, m: EpisodeMetrikleri) -> None:
        self.episode_listesi.append(m)

    def ozet(self) -> Dict[str, Any]:
        if not self.episode_listesi:
            return {}

        toplam = len(self.episode_listesi)
        basarili = sum(1 for e in self.episode_listesi if e.basarili)
        carpisma = sum(1 for e in self.episode_listesi if e.carpisma)
        hata = sum(1 for e in self.episode_listesi if e.sim_hatasi)

        ort_odul = sum(e.toplam_odul for e in self.episode_listesi) / toplam

        yakin_ihlal_toplam_adim = sum(e.yakin_ihlal_adim for e in self.episode_listesi)
        toplam_adim = sum(e.toplam_adim for e in self.episode_listesi)
        yakin_ihlal_yuzde = (yakin_ihlal_toplam_adim / toplam_adim * 100.0) if toplam_adim > 0 else 0.0

        basarili_sureler = [e.hedefe_ulasma_suresi for e in self.episode_listesi if e.basarili and e.hedefe_ulasma_suresi is not None]
        ort_sure = (sum(basarili_sureler) / len(basarili_sureler)) if basarili_sureler else None

        yol_ver = [e.yol_verimliligi() for e in self.episode_listesi if e.gercek_yol_uzunlugu > 0]
        ort_yol_ver = sum(yol_ver) / len(yol_ver) if yol_ver else 0.0

        sistem_kararliligi = ((toplam - hata) / toplam) * 100.0

        return {
            "toplam_episode": toplam,
            "basari_orani_yuzde": (basarili / toplam) * 100.0,
            "carpisma_orani_yuzde": (carpisma / toplam) * 100.0,
            "ortalama_odul": ort_odul,
            "yakin_mesafe_ihlali_yuzde": yakin_ihlal_yuzde,
            "ortalama_hedefe_ulasma_suresi": ort_sure,
            "ortalama_yol_verimliligi": ort_yol_ver,
            "sistem_kararliligi_yuzde": sistem_kararliligi,
        }

    def json_yaz(self, dosya: str) -> None:
        dizin_olustur(os.path.dirname(dosya))
        veri = {
            "ozet": self.ozet(),
            "episode_detaylari": [asdict(e) for e in self.episode_listesi],
        }
        with open(dosya, "w", encoding="utf-8") as f:
            json.dump(veri, f, ensure_ascii=False, indent=2)

    def csv_yaz(self, dosya: str) -> None:
        dizin_olustur(os.path.dirname(dosya))
        alanlar = list(asdict(self.episode_listesi[0]).keys()) if self.episode_listesi else []
        with open(dosya, "w", encoding="utf-8", newline="") as f:
            yaz = csv.DictWriter(f, fieldnames=alanlar)
            yaz.writeheader()
            for e in self.episode_listesi:
                yaz.writerow(asdict(e))
