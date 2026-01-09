from __future__ import annotations
import os
import json
import argparse
import yaml

try:
    import rclpy
except Exception:
    rclpy = None  # type: ignore

from .ajanlar.d3qn_ajan import D3QNAjan
from .ajanlar.davranis_agaci import GenisletilmisDavranisAgaci
from .ortamlar.ros2_gazebo_ortam import ROS2GazeboOrtam
from .yardimcilar import dizin_olustur, zaman_damgasi, EpisodeMetrikleri, MetrikToplayici

def argumanlari_ayarla() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    p.add_argument("--parametreler", required=True)
    p.add_argument("--senaryo", required=True)
    p.add_argument("--model", required=True, help="pth checkpoint yolu")
    p.add_argument("--episode", type=int, default=50)
    p.add_argument("--cikti_kok", default="test_sonuclari")
    return p

def main() -> None:
    args = argumanlari_ayarla().parse_args()

    with open(args.parametreler, "r", encoding="utf-8") as f:
        param = yaml.safe_load(f)
    with open(args.senaryo, "r", encoding="utf-8") as f:
        sen = yaml.safe_load(f)

    if rclpy is None:
        raise RuntimeError("ROS2 (rclpy) bulunamadi. Test ROS2 ortaminda calistirilmalidir.")

    y = param["yamlag"]
    d = param["davranis_agaci"]
    o = param["oduller"]

    bt = GenisletilmisDavranisAgaci(
        carpisma_esik=float(d["carpisma_esik"]),
        hedef_yaklasma_esik=float(d["hedef_yaklasma_esik"]),
        yakin_mesafe_esik=float(d["yakin_mesafe_esik"]),
    ) if bool(d.get("aktif", True)) else None

    ajan = D3QNAjan(
        durum_boyut=int(y["durum_boyut"]),
        aksiyon_boyut=int(y["aksiyon_boyut"]),
        gizli_boyut=int(y["gizli_boyut"]),
        ogrenme_orani=0.0001,
        gamma=float(param["egitim"]["gamma"]),
        epsilon_baslangic=0.0,
        epsilon_bitis=0.0,
        epsilon_azalma=1.0,
        bellek_boyutu=1,
        hedef_ag_guncelleme_adimi=1000000,
        hedef_ag_yumusak_guncelleme=0.0,
    )
    ajan.yukle(args.model)

    hedef = sen["yamlsenaryo"]["navigasyon"]["hedef"]
    hedef_xy = (float(hedef["x"]), float(hedef["y"]))

    ortam = ROS2GazeboOrtam(
        hedef_xy=hedef_xy,
        odul_param={
            "hedef_ulasildi": float(o["hedef_ulasildi"]),
            "carpisma": float(o["carpisma"]),
            "zaman_adimi": float(o["zaman_adimi"]),
            "ilerleme_katsayi": float(o["ilerleme_katsayi"]),
            "yakin_ihlal_cezasi": float(o["yakin_ihlal_cezasi"]),
        },
        bt=bt,
    )

    dizin = os.path.join(args.cikti_kok, f"{sen['yamlsenaryo']['isim']}_{zaman_damgasi()}")
    dizin_olustur(dizin)

    toplayici = MetrikToplayici()

    rclpy.init()
    try:
        for ep in range(1, args.episode + 1):
            toplam_odul = 0.0
            yakin_ihlal = 0
            durum, bilgi = ortam.sifirla()
            optimal = float(bilgi["hedefe_mesafe"])
            gercek_yol = 0.0

            basarili = False
            carpisma = False

            for adim in range(int(param["egitim"]["maksimum_adim_per_episode"])):
                if bt:
                    bt.guncelle(bilgi)
                    kisit = bt.aksiyon_kisitlama_al()
                else:
                    kisit = {}

                aksiyon = ajan.aksiyon_sec(durum, kesif_izinli=False)
                if aksiyon in kisit and kisit[aksiyon] >= 0.9:
                    aksiyon = 3 if aksiyon in (0,1,2) else 0

                sonraki, odul, bitti, bilgi = ortam.adim_at(aksiyon)
                toplam_odul += float(odul)

                if float(bilgi["minimum_mesafe"]) < float(param["davranis_agaci"]["yakin_mesafe_esik"]):
                    yakin_ihlal += 1

                lineer_hiz = float(bilgi.get("lineer_hiz", 0.0))
                gercek_yol += abs(lineer_hiz) * 0.1

                durum = sonraki

                if bitti:
                    if float(bilgi["hedefe_mesafe"]) < 0.35:
                        basarili = True
                    if float(bilgi["minimum_mesafe"]) < 0.25:
                        carpisma = True
                    break

            met = EpisodeMetrikleri(
                episode=ep, basarili=basarili, carpisma=carpisma, toplam_odul=float(toplam_odul),
                adim_sayisi=adim+1, hedefe_ulasma_suresi=None, yakin_ihlal_adim=int(yakin_ihlal),
                toplam_adim=int(adim+1), gercek_yol_uzunlugu=float(max(gercek_yol, 1e-6)),
                optimal_yol_uzunlugu=float(max(optimal, 1e-6)), sim_hatasi=False
            )
            toplayici.ekle(met)

        rapor = {
            "ozet": toplayici.ozet(),
            "detaylar": [met.__dict__ for met in toplayici.episode_listesi],
        }
        with open(os.path.join(dizin, "test_sonuclari.json"), "w", encoding="utf-8") as f:
            json.dump(rapor, f, ensure_ascii=False, indent=2)
    finally:
        rclpy.shutdown()
