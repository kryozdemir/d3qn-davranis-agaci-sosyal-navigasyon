#!/usr/bin/env python3
from enum import Enum
from typing import Dict, Any, List, Optional

import yaml


class DugumDurumu(Enum):
    BASARI = 1
    BASARISIZ = 2
    DEVAM = 3


class DavranisAgaci:
    def __init__(self, parametreler_yaml: str):
        self.parametreler_yaml = parametreler_yaml
        self.parametreler: Dict[str, Any] = {}
        self._yukle()

    def _yukle(self):
        if not self.parametreler_yaml:
            self.parametreler = {}
            return
        with open(self.parametreler_yaml, 'r', encoding='utf-8') as f:
            self.parametreler = yaml.safe_load(f) or {}

    def mod_belirle(self, kara_tahta: Dict[str, Any]) -> str:
        insan_var = bool(kara_tahta.get('insan_var', False))
        if insan_var:
            return 'sosyal'
        return 'normal'

    def kesif_yapilabilir_mi(self, kara_tahta: Dict[str, Any]) -> bool:
        mod = self.mod_belirle(kara_tahta)
        if mod == 'sosyal':
            return False
        return True

    def acikcil_mod_mu(self, kara_tahta: Dict[str, Any]) -> bool:
        return self.mod_belirle(kara_tahta) == 'normal'

    def hedef_modu_mu(self, kara_tahta: Dict[str, Any]) -> bool:
        hedef_mesafe = float(kara_tahta.get('hedef_mesafe', 999.0))
        esik = float(self.parametreler.get('davranis_agaci', {}).get('hedef_yaklasma_esik', 1.0))
        return hedef_mesafe <= esik


class GenisletilmisDavranisAgaci(DavranisAgaci):
    def __init__(self, parametreler_yaml: str):
        super().__init__(parametreler_yaml)
        self.kara_tahta: Dict[str, Any] = {}
        self.son_mod: str = 'normal'

        da = self.parametreler.get('davranis_agaci', {}) if isinstance(self.parametreler, dict) else {}
        self.carpisma_esik = float(da.get('carpisma_esik', 0.25))
        self.yakin_esik = float(da.get('yakin_mesafe_esik', 0.5)) if 'yakin_mesafe_esik' in da else 0.5

    def guncelle(self, kara_tahta: Dict[str, Any]):
        self.kara_tahta = dict(kara_tahta)
        self.son_mod = self.mod_belirle(self.kara_tahta)

    def kesif_izinli_mi(self) -> bool:
        return self.kesif_yapilabilir_mi(self.kara_tahta)

    def _izinli_aksiyonlari_hesapla(self) -> List[int]:
        minimum_mesafe = float(self.kara_tahta.get('minimum_mesafe', 999.0))
        hedef_aci = float(self.kara_tahta.get('hedef_aci', 0.0))

        if minimum_mesafe < self.carpisma_esik:
            return [3, 4]

        if minimum_mesafe < self.yakin_esik:
            if hedef_aci >= 0.0:
                return [2, 4]
            return [1, 3]

        return [0, 1, 2, 3, 4]

    def aksiyon_kisitlama_al(self) -> Dict[str, Any]:
        mod = self.son_mod
        izinli = self._izinli_aksiyonlari_hesapla()

        if mod == 'sosyal':
            return {
                'mod': 'sosyal',
                'maks_dogrusal_hiz': 0.16,
                'maks_acisal_hiz': 0.9,
                'ileri_izinli': True,
                'izinli_aksiyonlar': izinli
            }

        return {
            'mod': 'normal',
            'maks_dogrusal_hiz': 0.22,
            'maks_acisal_hiz': 1.2,
            'ileri_izinli': True,
            'izinli_aksiyonlar': izinli
        }
