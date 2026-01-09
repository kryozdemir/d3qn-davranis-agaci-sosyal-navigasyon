from __future__ import annotations
from typing import Optional, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener
except Exception:
    Node = object  # type: ignore
    Buffer = object  # type: ignore
    TransformListener = object  # type: ignore

class TF2Isleyici:
    def __init__(self, node: Node, hedef_cerceve: str = "odom", kaynak_cerceve: str = "base_link") -> None:
        self.node = node
        self.hedef_cerceve = hedef_cerceve
        self.kaynak_cerceve = kaynak_cerceve
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)

    def konum_al(self) -> Optional[Tuple[float, float]]:
        try:
            tf = self.buffer.lookup_transform(self.hedef_cerceve, self.kaynak_cerceve, rclpy.time.Time())
            return (float(tf.transform.translation.x), float(tf.transform.translation.y))
        except Exception:
            return None
