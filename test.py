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

class Grid:
    def __init__(self, width, depth, cell_size):
        self.width, self.depth, self.cell_size = width, depth, cell_size
        # build nodes
        self.nodes = [
            [ Node((i - width//2)*cell_size, (j - depth//2)*cell_size)
              for j in range(depth) ]
            for i in range(width)
        ]
        # link 4-way neighbors
        for i in range(width):
            for j in range(depth):
                n = self.nodes[i][j]
                for di, dj in ((1,0),(-1,0),(0,1),(0,-1)):
                    ni, nj = i+di, j+dj
                    if 0 <= ni < width and 0 <= nj < depth:
                        n.neighbors.append(self.nodes[ni][nj])

    def get_nearest_node(self, x, z):
        i = int(round(x/self.cell_size + self.width//2))
        j = int(round(z/self.cell_size + self.depth//2))
        i, j = clamp(i, 0, self.width-1), clamp(j, 0, self.depth-1)
        return self.nodes[i][j]

    def block_node_at_world_pos(self, pos):
        n = self.get_nearest_node(pos.x, pos.z)
        n.walkable = False

# --- A* implementation ---
def astar(start: Node, end: Node):
    open_set, closed_set = [start], set()
    # reset bookkeeping
    for row in grid.nodes:
        for n in row:
            n.g = n.h = n.f = 0
            n.parent = None

    while open_set:
        current = min(open_set, key=lambda n: n.f)
        if current is end:
            return reconstruct_path(end)
        open_set.remove(current)
        closed_set.add(current)

        for neigh in current.neighbors:
            if not neigh.walkable or neigh in closed_set:
                continue
            tentative_g = current.g + (current.pos - neigh.pos).length()
            if neigh not in open_set or tentative_g < neigh.g:
                neigh.parent = current
                neigh.g = tentative_g
                neigh.h = (neigh.pos - end.pos).length()
                neigh.f = neigh.g + neigh.h
                if neigh not in open_set:
                    open_set.append(neigh)
    return []

def reconstruct_path(end_node: Node):
    path = []
    n = end_node
    while n:
        path.append(n.pos)
        n = n.parent
    return list(reversed(path))

# --- Robot Entity that follows a path ---
class Robot(Entity):
    def __init__(self, **kwargs):
        super().__init__(model='cube', color=color.azure, scale=0.8, **kwargs)
        self.path = []
        self.speed = 5

    def start_pathfinding_to(self, target_node):
        start_node = grid.get_nearest_node(self.x, self.z)
        self.path = astar(start_node, target_node)

    def update(self):
        if self.path:
            target = self.path[0] + Vec3(0, self.scale_y/2, 0)
            direction = (target - self.position).normalized()
            step = direction * time.dt * self.speed
            if (self.position - target).length() <= step.length():
                self.position = target
                self.path.pop(0)
            else:
                self.position += step

# --- Main Ursina setup ---
if __name__ == '__main__':
    app = Ursina()

    # 1) Build a 21×21 grid covering -10…+10 in X and Z
    cell_size  = 1
    grid_width = grid_depth = 21
    grid = Grid(grid_width, grid_depth, cell_size)

    # 2) Draw an inner border
    border_thickness = 0.1
    half_span = (grid_width//2)*cell_size - cell_size/2
    # bottom edge
    Entity(model='cube',
           color=color.black,
           scale=(grid_width*cell_size, 0.2, border_thickness),
           position=(0, 0.1, -half_span))
    # top edge
    Entity(model='cube',
           color=color.black,
           scale=(grid_width*cell_size, 0.2, border_thickness),
           position=(0, 0.1, half_span))
    # left edge
    Entity(model='cube',
           color=color.black,
           scale=(border_thickness, 0.2, grid_depth*cell_size),
           position=(-half_span, 0.1, 0))
    # right edge
    Entity(model='cube',
           color=color.black,
           scale=(border_thickness, 0.2, grid_depth*cell_size),
           position=(half_span, 0.1, 0))

    # 3) Add a smaller, more compact ramp (to match the image you shared)
    ramp = Entity(model='cube',
                  color=color.gray,
                  scale=(3, 0.2, 0.8),  # Compact ramp size
                  rotation=Vec3(30, 0, 0),
                  position=Vec3(0, 0.1, 2),
                  collider='box')

    # 4) Ground plane (for raycasting clicks)
    ground = Entity(model='plane',
                    scale=(grid_width*cell_size, 1, grid_depth*cell_size),
                    collider='box',
                    color=color.light_gray)

    # 5) Example static obstacle
    obstacle = Entity(model='cube',
                      color=color.red,
                      scale=1,
                      position=Vec3(-3, 0.5, -2),
                      collider='box')
    grid.block_node_at_world_pos(obstacle.position)

    # 6) “Sun” in the corner
    sun = Entity(model='sphere',
                 color=color.yellow,
                 scale=0.5,
                 position=Vec3(-9, 1, -9))

    # 7) Start Point (Square with a Border)
    start_marker = Entity(model='cube',
                          color=color.yellow,
                          scale=(1, 0.1, 1),  # Square with border
                          position=Vec3(8, 0.1, 8))

    # 8) Robot placed at the start point
    robot = Robot(position=start_marker.position + Vec3(0, 0.5, 0))

    # 9) Endpoint marker (pink) hidden until click
    end_marker = Entity(model='sphere',
                        color=color.rgb(255,192,203),  # Pink color
                        scale=0.3,
                        visible=False)

    # 10) Handle mouse clicks: compute & show path
    def input(key):
        if key == 'left mouse down':
            hit = mouse.world_point
            if hit:
                tgt_node = grid.get_nearest_node(hit.x, hit.z)
                end_marker.position = Vec3(tgt_node.pos.x, 0.2, tgt_node.pos.z)
                end_marker.visible = True
                robot.start_pathfinding_to(tgt_node)

    # 11) Draw path as green segments
    def update():
        # clear old segments
        for e in scene.entities:
            if getattr(e, 'is_path_segment', False):
                destroy(e)
        # draw new ones
        for i in range(len(robot.path)-1):
            a, b = robot.path[i], robot.path[i+1]
            length = (b - a).length()
            yaw = degrees(atan2(b.x - a.x, b.z - a.z))
            seg = Entity(model='cube',
                         color=color.green,
                         scale=(0.05, 0.05, length),
                         position=(a + b)/2 + Vec3(0, 0.05, 0),
                         rotation=Vec3(0, yaw, 0))
            seg.is_path_segment = True

    Sky()
    app.run()


