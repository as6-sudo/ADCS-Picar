from .mechanical import Mechanical
from .efficient import Efficient
from .natural import Natural
from .erratic import Erratic

PROFILES = {p.key: p for p in (Mechanical, Efficient, Natural, Erratic)}
