import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from bluerov2_interfaces.srv import ExecuteRoutine

from .motion_primitives import MotionPrimitives
from .routines import Routines


class RoutineNode(Node):

    def __init__(self):

        super().__init__("routine_node")

        # ==========================================================
        # CORE COMPONENTS
        # ==========================================================

        self.controller = MotionPrimitives(self)
        self.routines = Routines(self.controller)

        # ==========================================================
        # STATE
        # ==========================================================

        self.is_executing = False

        # ==========================================================
        # SERVICE
        # ==========================================================

        self.srv = self.create_service(
            ExecuteRoutine,
            "execute_routine",
            self.service_callback
        )

        self.get_logger().info("RoutineNode ready")

    # ==========================================================
    # SERVICE CALLBACK (FAST + SAFE)
    # ==========================================================

    def service_callback(self, request, response):

        name = request.routine_name

        # ---- safety check ----
        if self.is_executing:
            response.success = False
            response.message = "A routine is already running"
            return response

        # ---- validation ----
        if name not in self.routines.registry:
            response.success = False
            response.message = f"Unknown routine: {name}"
            return response

        # ---- execute synchronously (controlled) ----
        self.is_executing = True

        self.get_logger().info(f"Executing routine: {name}")

        try:
            self.routines.run(name)

            response.success = True
            response.message = f"Completed: {name}"

        except Exception as e:

            self.get_logger().error(str(e))

            response.success = False
            response.message = str(e)

        finally:

            # always stop robot
            self.controller.stop()

            self.is_executing = False

        return response


# ==========================================================
# MAIN
# ==========================================================

def main():

    rclpy.init()

    node = RoutineNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    executor.spin()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()