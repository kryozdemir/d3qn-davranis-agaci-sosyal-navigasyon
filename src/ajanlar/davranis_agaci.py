from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Any, Optional, Tuple

class DugumDurumu(Enum):
    BASARI = 1
    BASARISIZ = 2
    DEVAM = 3

@dataclass
class KaraTahta:
    # sensör temelli özet
    minimum_mesafe: float = 999.0
    hedefe_mesafe: float = 999.0
    hedefe_aci: float = 0.0
    lineer_hiz: float = 0.0
    acisal_hiz: float = 0.0
    yakin_ihlal: bool = False

class DavranisAgaci:
    def __init__(self, carpisma_esik: float, hedef_yaklasma_esik: float, yakin_mesafe_esik: float) -> None:
        self.carpisma_esik = float(carpisma_esik)
        self.hedef_yaklasma_esik = float(hedef_yaklasma_esik)
        self.yakin_mesafe_esik = float(yakin_mesafe_esik)

    def mod_belirle(self, kt: KaraTahta) -> str:
        if kt.minimum_mesafe < self.carpisma_esik:
            return "acil_durum"
        if kt.hedefe_mesafe < self.hedef_yaklasma_esik:
            return "hedef_yaklasma"
        if kt.yakin_ihlal:
            return "sosyal"
        return "normal"

    def kesif_yapilabilir_mi(self, mod: str) -> bool:
        return mod in ("normal", "sosyal")

    def acikcil_mod_mu(self, mod: str) -> bool:
        return mod == "normal"

    def hedef_modu_mu(self, mod: str) -> bool:
        return mod in ("hedef_yaklasma",)

class GenisletilmisDavranisAgaci(DavranisAgaci):
    def __init__(self, carpisma_esik: float, hedef_yaklasma_esik: float, yakin_mesafe_esik: float) -> None:
        super().__init__(carpisma_esik, hedef_yaklasma_esik, yakin_mesafe_esik)
        self.kara_tahta = KaraTahta()
        self.mod: str = "normal"

    def guncelle(self, durum_bilgisi: Dict[str, Any]) -> None:
        self.kara_tahta.minimum_mesafe = float(durum_bilgisi.get("minimum_mesafe", self.kara_tahta.minimum_mesafe))
        self.kara_tahta.hedefe_mesafe = float(durum_bilgisi.get("hedefe_mesafe", self.kara_tahta.hedefe_mesafe))
        self.kara_tahta.hedefe_aci = float(durum_bilgisi.get("hedefe_aci", self.kara_tahta.hedefe_aci))
        self.kara_tahta.lineer_hiz = float(durum_bilgisi.get("lineer_hiz", self.kara_tahta.lineer_hiz))
        self.kara_tahta.acisal_hiz = float(durum_bilgisi.get("acisal_hiz", self.kara_tahta.acisal_hiz))

        self.kara_tahta.yakin_ihlal = self.kara_tahta.minimum_mesafe < self.yakin_mesafe_esik
        self.mod = self.mod_belirle(self.kara_tahta)

    def kesif_izinli_mi(self) -> bool:
        return self.kesif_yapilabilir_mi(self.mod)

    def aksiyon_kisitlama_al(self) -> Dict[int, float]:
        # D3QN aksiyonlarına yumuşak ceza katsayıları (0 iyi, 1 kötü gibi düşünülebilir)
        # acil durumda ileri aksiyonları ağır cezalandır
        if self.mod == "acil_durum":
            return {0: 1.0, 1: 1.0, 2: 1.0, 3: 0.0, 4: 0.0}
        # hedef yaklaşmada agresif dönmeyi biraz kısıtla
        if self.mod == "hedef_yaklasma":
            return {3: 0.4, 4: 0.4}
        # sosyal modda ileri+sol/sağ yerine yavaş düz veya kontrollü dönüş tercih edilsin
        if self.mod == "sosyal":
            return {1: 0.3, 2: 0.3}
        return {}
