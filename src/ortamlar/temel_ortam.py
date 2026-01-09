from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Tuple, Dict, Any
import numpy as np

class TemelOrtam(ABC):
    @abstractmethod
    def sifirla(self) -> Tuple[np.ndarray, Dict[str, Any]]:
        raise NotImplementedError

    @abstractmethod
    def adim_at(self, aksiyon: int) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        raise NotImplementedError
