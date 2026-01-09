from __future__ import annotations
import os
import random
from dataclasses import dataclass
from typing import Deque, Tuple, List, Optional, Dict, Any

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

class DuelingAg(nn.Module):
    def __init__(self, durum_boyut: int, aksiyon_boyut: int, gizli_boyut: int) -> None:
        super().__init__()
        self.ortak = nn.Sequential(
            nn.Linear(durum_boyut, gizli_boyut),
            nn.ReLU(),
            nn.Linear(gizli_boyut, gizli_boyut),
            nn.ReLU(),
        )
        self.deger = nn.Sequential(
            nn.Linear(gizli_boyut, gizli_boyut // 2),
            nn.ReLU(),
            nn.Linear(gizli_boyut // 2, 1),
        )
        self.avantaj = nn.Sequential(
            nn.Linear(gizli_boyut, gizli_boyut // 2),
            nn.ReLU(),
            nn.Linear(gizli_boyut // 2, aksiyon_boyut),
        )

    def forward(self, durum: torch.Tensor) -> torch.Tensor:
        oz = self.ortak(durum)
        v = self.deger(oz)
        a = self.avantaj(oz)
        q = v + (a - a.mean(dim=1, keepdim=True))
        return q

class DeneyimBellegi:
    def __init__(self, kapasite: int, durum_boyut: int) -> None:
        self.kapasite = int(kapasite)
        self.durum_boyut = int(durum_boyut)
        self.indeks = 0
        self.dolu = False

        self.durumlar = np.zeros((self.kapasite, self.durum_boyut), dtype=np.float32)
        self.aksiyonlar = np.zeros((self.kapasite,), dtype=np.int64)
        self.oduller = np.zeros((self.kapasite,), dtype=np.float32)
        self.sonraki_durumlar = np.zeros((self.kapasite, self.durum_boyut), dtype=np.float32)
        self.bitti = np.zeros((self.kapasite,), dtype=np.float32)

    def __len__(self) -> int:
        return self.kapasite if self.dolu else self.indeks

    def ekle(self, durum: np.ndarray, aksiyon: int, odul: float, sonraki_durum: np.ndarray, bitti: bool) -> None:
        i = self.indeks
        self.durumlar[i] = durum
        self.aksiyonlar[i] = int(aksiyon)
        self.oduller[i] = float(odul)
        self.sonraki_durumlar[i] = sonraki_durum
        self.bitti[i] = 1.0 if bitti else 0.0

        self.indeks = (self.indeks + 1) % self.kapasite
        if self.indeks == 0:
            self.dolu = True

    def ornekle(self, parti_boyutu: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        n = len(self)
        secimler = np.random.choice(n, size=parti_boyutu, replace=False)
        return (self.durumlar[secimler],
                self.aksiyonlar[secimler],
                self.oduller[secimler],
                self.sonraki_durumlar[secimler],
                self.bitti[secimler])

@dataclass
class EgitimIstatistikleri:
    kayip: float
    ort_q: float
    epsilon: float

class D3QNAjan:
    def __init__(
        self,
        durum_boyut: int,
        aksiyon_boyut: int,
        gizli_boyut: int,
        ogrenme_orani: float,
        gamma: float,
        epsilon_baslangic: float,
        epsilon_bitis: float,
        epsilon_azalma: float,
        bellek_boyutu: int,
        hedef_ag_guncelleme_adimi: int,
        hedef_ag_yumusak_guncelleme: float,
        cihaz: Optional[str] = None,
    ) -> None:
        self.durum_boyut = int(durum_boyut)
        self.aksiyon_boyut = int(aksiyon_boyut)
        self.gizli_boyut = int(gizli_boyut)

        self.gamma = float(gamma)
        self.epsilon = float(epsilon_baslangic)
        self.epsilon_bitis = float(epsilon_bitis)
        self.epsilon_azalma = float(epsilon_azalma)

        self.hedef_ag_guncelleme_adimi = int(hedef_ag_guncelleme_adimi)
        self.tau = float(hedef_ag_yumusak_guncelleme)

        self.cihaz = torch.device(cihaz if cihaz else ("cuda" if torch.cuda.is_available() else "cpu"))

        self.politika_ag = DuelingAg(self.durum_boyut, self.aksiyon_boyut, self.gizli_boyut).to(self.cihaz)
        self.hedef_ag = DuelingAg(self.durum_boyut, self.aksiyon_boyut, self.gizli_boyut).to(self.cihaz)
        self.hedef_ag.load_state_dict(self.politika_ag.state_dict())
        self.hedef_ag.eval()

        self.optimizleyici = optim.Adam(self.politika_ag.parameters(), lr=float(ogrenme_orani))
        self.kayip_fonksiyonu = nn.SmoothL1Loss()

        self.bellek = DeneyimBellegi(bellek_boyutu, self.durum_boyut)

        self.kayip_gecmisi: List[float] = []
        self.adim_sayaci = 0

    def aksiyon_sec(self, durum: np.ndarray, kesif_izinli: bool = True) -> int:
        if kesif_izinli and random.random() < self.epsilon:
            return random.randrange(self.aksiyon_boyut)

        with torch.no_grad():
            t = torch.as_tensor(durum, dtype=torch.float32, device=self.cihaz).unsqueeze(0)
            q = self.politika_ag(t)
            return int(torch.argmax(q, dim=1).item())

    def gecis_kaydet(self, durum: np.ndarray, aksiyon: int, odul: float, sonraki_durum: np.ndarray, bitti: bool) -> None:
        self.bellek.ekle(durum, aksiyon, odul, sonraki_durum, bitti)

    def epsilon_guncelle(self) -> None:
        if self.epsilon > self.epsilon_bitis:
            self.epsilon *= self.epsilon_azalma
            if self.epsilon < self.epsilon_bitis:
                self.epsilon = self.epsilon_bitis

    def _hedef_ag_guncelle(self) -> None:
        # yumuşak güncelleme
        with torch.no_grad():
            for hedef_p, politika_p in zip(self.hedef_ag.parameters(), self.politika_ag.parameters()):
                hedef_p.data.mul_(1.0 - self.tau)
                hedef_p.data.add_(self.tau * politika_p.data)

    def egit_adim(self, parti_boyutu: int) -> Optional[EgitimIstatistikleri]:
        if len(self.bellek) < parti_boyutu:
            return None

        durumlar, aksiyonlar, oduller, sonraki_durumlar, bitti = self.bellek.ornekle(parti_boyutu)

        durumlar_t = torch.as_tensor(durumlar, dtype=torch.float32, device=self.cihaz)
        aksiyonlar_t = torch.as_tensor(aksiyonlar, dtype=torch.int64, device=self.cihaz).unsqueeze(1)
        oduller_t = torch.as_tensor(oduller, dtype=torch.float32, device=self.cihaz).unsqueeze(1)
        sonraki_t = torch.as_tensor(sonraki_durumlar, dtype=torch.float32, device=self.cihaz)
        bitti_t = torch.as_tensor(bitti, dtype=torch.float32, device=self.cihaz).unsqueeze(1)

        # Double DQN: aksiyonu politika seçer, hedef ağ değerlendirir
        q_mevcut = self.politika_ag(durumlar_t).gather(1, aksiyonlar_t)

        with torch.no_grad():
            sonraki_aksiyon = torch.argmax(self.politika_ag(sonraki_t), dim=1, keepdim=True)
            q_sonraki = self.hedef_ag(sonraki_t).gather(1, sonraki_aksiyon)
            hedef_q = oduller_t + (1.0 - bitti_t) * self.gamma * q_sonraki

        kayip = self.kayip_fonksiyonu(q_mevcut, hedef_q)

        self.optimizleyici.zero_grad()
        kayip.backward()
        torch.nn.utils.clip_grad_norm_(self.politika_ag.parameters(), 10.0)
        self.optimizleyici.step()

        self.kayip_gecmisi.append(float(kayip.item()))
        self.adim_sayaci += 1

        if self.adim_sayaci % self.hedef_ag_guncelleme_adimi == 0:
            self._hedef_ag_guncelle()

        ort_q = float(q_mevcut.detach().mean().item())
        return EgitimIstatistikleri(kayip=float(kayip.item()), ort_q=ort_q, epsilon=float(self.epsilon))

    def kaydet(self, dosya: str) -> None:
        os.makedirs(os.path.dirname(dosya), exist_ok=True)
        torch.save({
            "politika_ag": self.politika_ag.state_dict(),
            "hedef_ag": self.hedef_ag.state_dict(),
            "optimizleyici": self.optimizleyici.state_dict(),
            "epsilon": self.epsilon,
            "adim_sayaci": self.adim_sayaci,
        }, dosya)

    def yukle(self, dosya: str) -> None:
        veri = torch.load(dosya, map_location=self.cihaz)
        self.politika_ag.load_state_dict(veri["politika_ag"])
        self.hedef_ag.load_state_dict(veri["hedef_ag"])
        self.optimizleyici.load_state_dict(veri["optimizleyici"])
        self.epsilon = float(veri.get("epsilon", self.epsilon))
        self.adim_sayaci = int(veri.get("adim_sayaci", 0))

    def istatistikler_al(self) -> Dict[str, Any]:
        return {
            "epsilon": self.epsilon,
            "adim_sayaci": self.adim_sayaci,
            "ortalama_kayip": float(np.mean(self.kayip_gecmisi)) if self.kayip_gecmisi else None,
            "son_kayip": self.kayip_gecmisi[-1] if self.kayip_gecmisi else None,
        }
