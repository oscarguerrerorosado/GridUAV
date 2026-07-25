"""Movement pattern implementations for UAV navigation."""

from RobotControl.navigation_patterns.base import NavigationPattern
from RobotControl.navigation_patterns.circle import CirclePattern
from RobotControl.navigation_patterns.random_2d import RandomExploration2D
from RobotControl.navigation_patterns.random_3d import RandomExploration3D
from RobotControl.navigation_patterns.straight_line import StraightLinePattern

PATTERN_REGISTRY = {
    "straight_line": StraightLinePattern,
    "circle": CirclePattern,
    "random_2d": RandomExploration2D,
    "random_3d": RandomExploration3D,
}

__all__ = [
    "NavigationPattern",
    "StraightLinePattern",
    "CirclePattern",
    "RandomExploration2D",
    "RandomExploration3D",
    "PATTERN_REGISTRY",
]
