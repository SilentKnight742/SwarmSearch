from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any


@dataclass
class SwarmEvent:
    event_type: str
    source: str
    message: str

    data: dict[str, Any] = field(default_factory=dict)

    timestamp: str = field(
        default_factory=lambda: (
            datetime.now(timezone.utc).isoformat()
        )
    )