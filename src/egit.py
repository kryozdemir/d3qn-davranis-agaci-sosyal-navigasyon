from __future__ import annotations
import os
import time
import argparse
from typing import Dict, Any, Tuple, Optional
import yaml
import numpy as np

from torch.utils.tensorboard import SummaryWriter

try:
    import rclpy
except Exception:
    rclpy = None  # type: ignore

from .ajanlar.d3qn_ajan import D3QNAjan
from .ajanlar.davranis_agaci import GenisletilmisDavranisAgaci
from .ortamlar.ros2_gazebo_ortam import ROS2GazeboOrtam
from .yardimcilar import dizin_olustur, zaman_damgasi, EpisodeMetrikleri, MetrikToplayici

class EgitimYoneticisi:
    def __init__(self, parametreler: Dict[str, Any], senaryo: Dict[str, Any], cikti_kok: str, cihaz: Optional[str] = None) -> None:
        self.parametreler = parametreler
        self.senaryo = senaryo

        self.cikti_kok = cikti_kok
        self.deney_adi = f"{senaryo['yamlsenaryo']['isim']}_{zaman_damgasi()}"
        self.deney_dizini = os.path.join(cikti_kok, self.deney_adi)
        self.kontrol_dizini = os.path.join(self.deney_dizini, "kontrol_noktalari")
        self.log_dizini = os.path.join(self.deney_dizini, "loglar")
        self.rapor_dizini = os.path.join(self.deney_dizini, "raporlar")

        self.dizinleri_olustur()

        y = parametreler["yamlag"]
        e = parametreler["egitim"]
        d = parametreler["davranis_agaci"]
        o = parametreler["oduller"]

        self.bt = GenisletilmisDavranisAgaci(
            carpisma_esik=float(d["carpisma_esik"]),
            hedef_yaklasma_esik=float(d["hedef_yaklasma_esik"]),
            yakin_mesafe_esik=float(d["yakin_mesafe_esik"]),
        ) if bool(d.get("aktif", True)) else None

        self.ajan = D3QNAjan(
            durum_boyut=int(y["durum_boyut"]),
            aksiyon_boyut=int(y["aksiyon_boyut"]),
            gizli_boyut=int(y["gizli_boyut"]),
            ogrenme_orani=float(e["ogrenme_orani"]),
            gamma=float(e["gamma"]),
            epsilon_baslangic=float(e["epsilon_baslangic"]),
            epsilon_bitis=float(e["epsilon_bitis"]),
            epsilon_azalma=float(e["epsilon_azalma"]),
            bellek_boyutu=int(e["bellek_boyutu"]),
            hedef_ag_guncelleme_adimi=int(e["hedef_ag_guncelleme_adimi"]),
            hedef_ag_yumusak_guncelleme=float(e["hedef_ag_yumusak_guncelleme"]),
            cihaz=cihaz,
        )

        nav = senaryo["yamlsenaryo"]["navigasyon"]
        hedef = nav["hedef"]
        hedef_xy = (float(hedef["x"]), float(hedef["y"]))

        # robot parametreleri aynı klasörde beklenir
        self.robot_param = None

        self.ortam = ROS2GazeboOrtam(
            hedef_xy=hedef_xy,
            odul_param={
                "hedef_ulasildi": float(o["hedef_ulasildi"]),
                "carpisma": float(o["carpisma"]),
                "zaman_adimi": float(o["zaman_adimi"]),
                "ilerleme_katsayi": float(o["ilerleme_katsayi"]),
                "yakin_ihlal_cezasi": float(o["yakin_ihlal_cezasi"]),
            },
            bt=self.bt,
        )

        self.writer = SummaryWriter(log_dir=self.log_dizini)
        self.toplayici = MetrikToplayici()

        self.episode_sayisi = int(e["episode_sayisi"])
        self.maks_adim = int(e["maksimum_adim_per_episode"])
        self.parti_boyutu = int(e["parti_boyutu"])
        self.baslangic_rastgele = int(e.get("baslangic_rastgele_adim", 0))
        self.kontrol_aralik = 100

    def dizinleri_olustur(self) -> None:
        dizin_olustur(self.deney_dizini)
        dizin_olustur(self.kontrol_dizini)
        dizin_olustur(self.log_dizini)
        dizin_olustur(self.rapor_dizini)

    def episode_egit(self, ep: int) -> Tuple[float, EpisodeMetrikleri]:
        bas_zaman = time.time()
        yakin_ihlal_adim = 0
        toplam_odul = 0.0
        carpisma = False
        basarili = False

        durum, bilgi = self.ortam.sifirla()
        onceki_xy = None
        gercek_yol = 0.0

        # optimal yol (düz çizgi yaklaşık)
        optimal = float(bilgi["hedefe_mesafe"])

        for adim in range(self.maks_adim):
            # BT güncelle ve kesif izni
            if self.bt:
                self.bt.guncelle(bilgi)
                kesif_izinli = self.bt.kesif_izinli_mi()
                kisit = self.bt.aksiyon_kisitlama_al()
            else:
                kesif_izinli = True
                kisit = {}

            # aksiyon seç
            if ep * self.maks_adim + adim < self.baslangic_rastgele:
                aksiyon = np.random.randint(0, self.ajan.aksiyon_boyut)
            else:
                aksiyon = self.ajan.aksiyon_sec(durum, kesif_izinli=kesif_izinli)

            # kısıtlar (sert)
            if aksiyon in kisit and kisit[aksiyon] >= 0.9:
                aksiyon = 3 if aksiyon in (0,1,2) else 0

            sonraki_durum, odul, bitti, bilgi = self.ortam.adim_at(aksiyon)

            # yakın ihlal
            if float(bilgi["minimum_mesafe"]) < float(self.parametreler["davranis_agaci"]["yakin_mesafe_esik"]):
                yakin_ihlal_adim += 1

            toplam_odul += float(odul)

            # yol uzunluğu (odom pozisyonuna erişim sınırlı: hızdan yaklaşık)
            # daha güvenli: durum vektöründeki hızlarla dt üzerinden yaklaşık
            lineer_hiz = float(bilgi.get("lineer_hiz", 0.0))
            gercek_yol += abs(lineer_hiz) * 0.1  # timer dt

            self.ajan.gecis_kaydet(durum, aksiyon, float(odul), sonraki_durum, bitti)

            ist = self.ajan.egit_adim(self.parti_boyutu)
            if ist is not None:
                self.writer.add_scalar("kayip", ist.kayip, ep * self.maks_adim + adim)
                self.writer.add_scalar("ortalama_q", ist.ort_q, ep * self.maks_adim + adim)
                self.writer.add_scalar("epsilon", ist.epsilon, ep * self.maks_adim + adim)

            durum = sonraki_durum

            if bitti:
                # hedef mi çarpışma mı
                if float(bilgi["hedefe_mesafe"]) < 0.35:
                    basarili = True
                if float(bilgi["minimum_mesafe"]) < 0.25:
                    carpisma = True
                break

        self.ajan.epsilon_guncelle()

        sure = time.time() - bas_zaman
        met = EpisodeMetrikleri(
            episode=ep,
            basarili=basarili,
            carpisma=carpisma,
            toplam_odul=float(toplam_odul),
            adim_sayisi=adim + 1,
            hedefe_ulasma_suresi=float(sure) if basarili else None,
            yakin_ihlal_adim=int(yakin_ihlal_adim),
            toplam_adim=int(adim + 1),
            gercek_yol_uzunlugu=float(max(gercek_yol, 1e-6)),
            optimal_yol_uzunlugu=float(max(optimal, 1e-6)),
            sim_hatasi=False,
        )
        return float(toplam_odul), met

    def kontrol_noktasi_kaydet(self, ep: int) -> None:
        dosya = os.path.join(self.kontrol_dizini, f"kontrol_noktasi_ep{ep}.pth")
        self.ajan.kaydet(dosya)

    def rapor_yaz(self) -> None:
        self.toplayici.json_yaz(os.path.join(self.rapor_dizini, "metrik_raporu.json"))
        self.toplayici.csv_yaz(os.path.join(self.rapor_dizini, "episode_metrikleri.csv"))

    def egit(self) -> None:
        for ep in range(1, self.episode_sayisi + 1):
            try:
                toplam_odul, met = self.episode_egit(ep)
                self.toplayici.ekle(met)

                self.writer.add_scalar("episode_toplam_odul", toplam_odul, ep)
                self.writer.add_scalar("basari", 1.0 if met.basarili else 0.0, ep)
                self.writer.add_scalar("carpisma", 1.0 if met.carpisma else 0.0, ep)
                self.writer.add_scalar("yakin_ihlal_yuzde", met.yakin_ihlal_orani_yuzde(), ep)
                self.writer.add_scalar("yol_verimliligi", met.yol_verimliligi(), ep)

                if ep % self.kontrol_aralik == 0 or ep == self.episode_sayisi:
                    self.kontrol_noktasi_kaydet(ep)
                    self.rapor_yaz()

                if ep % 10 == 0:
                    ozet = self.toplayici.ozet()
                    print(f"Episode {ep}/{self.episode_sayisi} | ort_odul={ozet.get('ortalama_odul'):.2f} | basari%={ozet.get('basari_orani_yuzde'):.1f} | carpisma%={ozet.get('carpisma_orani_yuzde'):.1f} | yakin_ihlal%={ozet.get('yakin_mesafe_ihlali_yuzde'):.1f}")

            except Exception:
                # sistem kararlılığı metriği için sim hatası işaretle
                met = EpisodeMetrikleri(
                    episode=ep, basarili=False, carpisma=False, toplam_odul=0.0, adim_sayisi=0,
                    hedefe_ulasma_suresi=None, yakin_ihlal_adim=0, toplam_adim=0,
                    gercek_yol_uzunlugu=0.0, optimal_yol_uzunlugu=0.0, sim_hatasi=True
                )
                self.toplayici.ekle(met)
                self.rapor_yaz()

        self.writer.flush()
        self.writer.close()

def argumanlari_ayarla() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    p.add_argument("--parametreler", required=True, help="d3qn_parametreler.yaml yolu")
    p.add_argument("--senaryo", required=True, help="senaryo yaml yolu")
    p.add_argument("--cikti_kok", default="egitim_ciktilar", help="egitim cikti kok dizini")
    p.add_argument("--cihaz", default=None, help="cuda/cpu gibi cihaz secimi (opsiyonel)")
    return p

def main() -> None:
    args = argumanlari_ayarla().parse_args()

    with open(args.parametreler, "r", encoding="utf-8") as f:
        parametreler = yaml.safe_load(f)
    with open(args.senaryo, "r", encoding="utf-8") as f:
        senaryo = yaml.safe_load(f)

    if rclpy is None:
        raise RuntimeError("ROS2 (rclpy) bulunamadi. Egitim ROS2 ortaminda calistirilmalidir.")

    rclpy.init()
    try:
        yon = EgitimYoneticisi(parametreler, senaryo, cikti_kok=args.cikti_kok, cihaz=args.cihaz)
        yon.egit()
    finally:
        rclpy.shutdown()
