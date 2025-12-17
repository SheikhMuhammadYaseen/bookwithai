---
id: ai-planning
title: Planning
sidebar_label: Planning
sidebar_position: 2
---

## Understanding Planning in Physical AI {#ch7-sec2-understanding-planning}

**Planning** in Physical AI refers to the computational process of determining a sequence of actions that will achieve a specific goal, while considering constraints, uncertainties, and the dynamics of the physical world. Planning bridges the gap between perception (understanding the world) and action (changing the world), enabling robots to make intelligent decisions about how to accomplish tasks.

### Types of Planning in Robotics: {#ch7-sec2-types-planning}

1.  **Task Planning**: High-level planning of what tasks to perform (e.g., "make coffee")
2.  **Motion Planning**: Determining how to move from one configuration to another
3.  **Path Planning**: Finding collision-free paths through the environment
4.  **Trajectory Planning**: Generating smooth, time-parameterized motions
5.  **Reactive Planning**: Real-time response to unexpected events
6.  **Hierarchical Planning**: Combining multiple planning levels

### Planning Pipeline Architecture: {#ch7-sec2-pipeline-architecture}

```python
import rclpy
from clpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
import numpy as np, heapq

class MiniPlanner(Node):
    def __init__(self):
        super().__init__('mini_planner')
        self.goal = None; self.pose = None; self.grid = None

        self.create_subscription(PoseStamped, '/goal', self.goal_cb, 10)
        self.create_subscription(PoseStamped, '/pose', self.pose_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.pub = self.create_publisher(Path, '/path', 10)
        self.timer = self.create_timer(0.5, self.plan)

    def goal_cb(self, m): self.goal = m.pose.position
    def pose_cb(self, m): self.pose = m.pose.position
    def map_cb(self, m):
        self.map = m; d = np.array(m.data).reshape(m.info.height,-1)
        self.grid = d > 50

    def plan(self):
        if not (self.goal and self.pose and self.grid is not None): return
        start = self.to_grid(self.pose); goal = self.to_grid(self.goal)
        path = self.a_star(start, goal)
        if path: self.pub.publish(self.to_ros(path)); self.goal = None

    def a_star(self, s, g):
        if not self.free(s) or not self.free(g): return None
        Q=[(0,s)]; from_p={}; g_cost={s:0}
        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            pass
        while Q:
            _,c=heapq.heappop(Q)
            if c==g: return self.reconstruct(from_p,c)
            for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                n=(c[0]+dx,c[1]+dy)
                if not self.free(n): continue
                ng=g_cost[c]+1
                if n not in g_cost or ng<g_cost[n]:
                    from_p[n]=c; g_cost[n]=ng
                    f=ng+np.hypot(n[0]-g[0],n[1]-g[1])
                    heapq.heappush(Q,(f,n))
        return None

    def free(self,p):
        h,w=self.grid.shape
        return 0<=p[0]<w and 0<=p[1]<h and not self.grid[p[1],p[0]]

    def reconstruct(self,src,c):
        path=[c]
        while c in src: c=src[c]; path.append(c)
        return path[::-1]

    def to_grid(self,p):
        m=self.map.info; o=m.origin.position
        return (int((p.x-o.x)/m.resolution),
                int((p.y-o.y)/m.resolution))

    def to_ros(self,path):
        P=Path(); P.header.frame_id='map'
        m=self.map.info; o=m.origin.position; r=m.resolution
        for (x,y) in path:
            ps=PoseStamped(); ps.pose.position.x=o.x+x*r
            ps.pose.position.y=o.y+y*r; ps.pose.orientation.w=1
            P.poses.append(ps)
        return P

def main():
    rclpy.init(); rclpy.spin(MiniPlanner()); rclpy.shutdown()

if __name__=='__main__': main()

```

### Task Planning with PDDL: {#ch7-sec2-pddl-planning}

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String
import numpy as np
import heapq

class PlanningSystem(Node):
    """Lightweight Task + Motion Planning System."""

    def __init__(self):
        super().__init__('planning_system')
        self.goal = None
        self.grid = None
        self.robot_pose = None

        # Communication setup
        self.create_subscription(PoseStamped, '/planning/goal', self.goal_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.create_subscription(PoseStamped, '/robot_pose', self.robot_cb, 10)

        self.path_pub = self.create_publisher(Path, '/planning/path', 10)
        self.status_pub = self.create_publisher(String, '/planning/status', 10)

        self.timer = self.create_timer(0.5, self.planning_loop)
        self.get_logger().info("Planning System Ready")

    def goal_cb(self, msg):
        self.goal = msg.pose.position
        self.publish_status("Goal Received")

    def map_cb(self, msg):
        self.grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width) > 50
        self.map_info = msg.info

    def robot_cb(self, msg):
        self.robot_pose = msg.pose.position

    def planning_loop(self):
        if self.goal and self.grid is not None and self.robot_pose:
            start = self.to_grid(self.robot_pose)
            goal = self.to_grid(self.goal)
            path = self.a_star(start, goal)

            if path:
                self.path_pub.publish(self.to_ros_path(path))
                self.publish_status("Path Published")
                self.goal = None

    # ------------------ A* Algorithm ------------------ #
    def a_star(self, start, goal):
        if not self.valid(start) or not self.valid(goal):
            return None

        open_set = [(0, start)]
        came_from, g = {}, {start: 0}
        dirs = [(1,0),(-1,0),(0,1),(0,-1)]

        while open_set:
            _, cur = heapq.heappop(open_set)
            if cur == goal:
                return self.reconstruct(came_from, cur)

            for dx, dy in dirs:
                nxt = (cur[0]+dx, cur[1]+dy)
                if not self.valid(nxt): continue
                new_g = g[cur] + 1
                if nxt not in g or new_g < g[nxt]:
                    came_from[nxt] = cur
                    g[nxt] = new_g
                    f = new_g + np.linalg.norm(np.subtract(nxt, goal))
                    heapq.heappush(open_set, (f, nxt))
        return None

    def valid(self, p):
        h, w = self.grid.shape
        return 0 <= p[0] < w and 0 <= p[1] < h and not self.grid[p[1], p[0]]

    def reconstruct(self, came_from, cur):
        path = [cur]
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)
        return list(reversed(path))

    # ------------------ Conversions ------------------ #
    def to_grid(self, pos):
        x = int((pos.x - self.map_info.origin.position.x) / self.map_info.resolution)
        y = int((pos.y - self.map_info.origin.position.y) / self.map_info.resolution)
        return (x, y)

    def to_world(self, p):
        pose = PoseStamped()
        pose.pose.position.x = p[0] * self.map_info.resolution + self.map_info.origin.position.x
        pose.pose.position.y = p[1] * self.map_info.resolution + self.map_info.origin.position.y
        pose.pose.orientation.w = 1.0
        return pose

    def to_ros_path(self, path):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.poses = [self.to_world(p) for p in path]
        return msg

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))

def main():
    rclpy.init()
    node = PlanningSystem()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

### Key Planning Algorithms: {#ch7-sec2-key-algorithms}

*   **A***: Optimal path planning with heuristics
*   **D***: Dynamic A* for replanning in changing environments
*   **RRT/RRT***: Sampling-based planning for high-dimensional spaces
*   **PRM**: Probabilistic Roadmap for multi-query planning
*   **Potential Fields**: Gradient-based reactive planning
*   **MPC**: Model Predictive Control for constrained optimization
*   **Behavior Trees**: Modular task planning and execution

### References and Further Reading: {#ch7-sec2-references}

*   **Book**: *[Principles of Robot Motion: Theory, Algorithms, and Implementations](https://mitpress.mit.edu/books/principles-robot-motion)* by Howie Choset et al.
*   **Research**: *[Rapidly-exploring Random Trees: A New Tool for Path Planning](https://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf)* by Lavalle
*   **Library**: [OMPL (Open Motion Planning Library)](https://ompl.kavrakilab.org/) for motion planning algorithms
*   **Framework**: [MoveIt](https://moveit.ros.org/) for ROS-based motion planning
*   **Paper**: *[CHOMP: Gradient Optimization Techniques for Efficient Motion Planning](https://www.ri.cmu.edu/pub_files/2009/5/choom_iros09.pdf)* for trajectory optimization
*   **Tutorial**: [ROS2 Navigation Stack](https://navigation.ros.org/) for practical implementation
*   **Course**: [Robot Motion Planning](https://www.coursera.org/learn/robotics-motion-planning) on Coursera

### Future Directions: {#ch7-sec2-future-directions}

*   **Learning-based Planning**: Using neural networks for planning
*   **Interactive Planning**: Human-in-the-loop planning systems
*   **Explainable Planning**: Understanding why plans were generated
*   **Multi-modal Planning**: Planning across different motion primitives
*   **Lifelong Planning**: Continuous planning and adaptation
*   **Distributed Planning**: Planning in decentralized systems
