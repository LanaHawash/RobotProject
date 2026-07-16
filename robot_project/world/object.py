from dataclasses import dataclass
from typing import Optional


@dataclass
class WorldObject:
    object_id: int
    label: str
    confidence: float
    distance_mm: Optional[int]
    center_x: int
    center_y: int
