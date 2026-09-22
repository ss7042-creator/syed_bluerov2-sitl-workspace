import random
from .motion_primitives import MotionPrimitives


class Routines:
    """
    High-level behavior routines built from MotionPrimitives.

    Available routines:
        square        - 4-sided square pattern in the horizontal plane
        circle        - circular arc approximated by forward + incremental yaw steps
        random        - structured random exploration with depth changes
        random_drift  - slower, unpredictable drift-and-scan pattern
        dive          - descend, traverse, resurface
        spin          - 360° scan in place
    """

    def __init__(self, controller: MotionPrimitives):

        self.c = controller

        # ==========================================================
        # ROUTINE LOOKUP TABLE
        # ==========================================================

        self.registry = {
            "square":       self.square,
            "circle":       self.circle,
            "random":       self.random_explore,
            "dive":         self.dive_surface,
            "spin":         self.spin_scan,
        }

    # ==========================================================
    # ROUTINE DISPATCHER
    # ==========================================================

    def run(self, name: str):
        routine = self.registry.get(name)
        if routine is None:
            raise ValueError(
                f"Unknown routine: '{name}'. "
                f"Available: {list(self.registry.keys())}"
            )
        routine()

    # ==========================================================
    # ROUTINE 1 - SQUARE PATTERN
    # ==========================================================

    def square(self, side_duration=2.5, speed=0.5, turn_duration=1.4, turn_speed=0.5):
        """
        Travel 4 sides of a square, rotating 90° at each corner.
        Tune turn_duration for your vehicle to achieve a clean 90°.
        """
        self.c.node.get_logger().info("Routine: square")

        for side in range(4):
            self.c.node.get_logger().info(f"  Square side {side + 1}/4")
            self.c.forward(side_duration, speed)
            self.c.rotate_right(turn_duration, turn_speed)

    # ==========================================================
    # ROUTINE 2 - CIRCLE PATTERN
    # ==========================================================

    def circle(self, steps=16, step_duration=0.8, speed=0.5, yaw_duration=0.35, yaw_speed=0.5):
        """
        Approximate a circle by interleaving forward motion with small
        yaw increments. steps=16 gives 16 × 22.5° ≈ 360°.
        Increase steps for a smoother arc.
        """
        self.c.node.get_logger().info("Routine: circle")

        for step in range(steps):
            self.c.node.get_logger().info(f"  Circle step {step + 1}/{steps}")
            self.c.forward(step_duration, speed)
            self.c.rotate_right(yaw_duration, yaw_speed)

    # ==========================================================
    # ROUTINE 3 - RANDOM EXPLORATION
    # ==========================================================

    def random_explore(self, moves=8):
        """
        Randomized exploration: picks random primitives, speeds, and
        durations to cover unknown space. Includes occasional depth changes.
        """
        self.c.node.get_logger().info("Routine: random_explore")

        horizontal = [
            self.c.forward,
            self.c.backward,
            self.c.left,
            self.c.right,
        ]
        rotations = [
            self.c.rotate_left,
            self.c.rotate_right,
        ]
        vertical = [
            self.c.ascend,
            self.c.descend,
        ]

        for i in range(moves):
            roll = random.random()

            if roll < 0.50:
                # 50% — move horizontally
                action = random.choice(horizontal)
                duration = random.uniform(1.0, 3.0)
                speed = random.uniform(0.4, 0.8)
            elif roll < 0.75:
                # 25% — rotate
                action = random.choice(rotations)
                duration = random.uniform(0.8, 2.0)
                speed = random.uniform(0.3, 0.6)
            else:
                # 25% — vertical
                action = random.choice(vertical)
                duration = random.uniform(1.0, 2.5)
                speed = random.uniform(0.3, 0.6)

            self.c.node.get_logger().info(
                f"  random_explore [{i+1}/{moves}]: "
                f"{action.__name__} duration={duration:.1f}s speed={speed:.2f}"
            )
            action(duration, speed)


    # ==========================================================
    # ROUTINE 4 - DIVE AND SURFACE
    # ==========================================================

    def dive_surface(self, depth_duration=3.0, traverse_duration=2.0,
                     depth_speed=0.5, traverse_speed=0.4):
        """
        Descend to depth, traverse forward, then resurface.
        """
        self.c.node.get_logger().info("Routine: dive_surface")

        self.c.descend(depth_duration, depth_speed)
        self.c.forward(traverse_duration, traverse_speed)
        self.c.ascend(depth_duration, depth_speed)

    # ==========================================================
    # ROUTINE 5 - SPIN SCAN
    # ==========================================================

    def spin_scan(self, duration=7.0, speed=0.3):
        """
        Slow 360° rotation in place for situational awareness / scanning.
        Default duration at speed=0.3 should cover a full rotation;
        tune duration for your vehicle.
        """
        self.c.node.get_logger().info("Routine: spin_scan")

        self.c.rotate_left(duration, speed)