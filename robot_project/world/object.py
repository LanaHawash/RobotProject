from dataclasses import dataclass


@dataclass
class WorldObject:
    object_id: int
    label: str
    confidence: float
    distance_mm: int
    center_x: int
    center_y: int