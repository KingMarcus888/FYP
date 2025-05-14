from ursina import *
from math import atan2, degrees

# --- Node & Grid definitions for A* ---
class Node:
    def __init__(self, x, z, walkable=True):
        self.pos       = Vec3(x, 0, z)
        self.walkable  = walkable
        self.neighbors = []
        self.g = self.h = self.f = 0
        self.parent   = None
