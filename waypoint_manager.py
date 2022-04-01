


class WaypointManager:
    def __init__(self, waypoints, controller):
        self.waypoints = waypoints
        self.controller = controller
        self.current_waypoint_index = -1
        self.paused = False
        self.all_waypoints_reached = False

    def pause(self):
        self.paused = True
        self.controller.cancel()

    def unpause(self):
        self.paused = False
        self.go_to_waypoint(self.current_waypoint_index)

    def go_to_waypoint(self, index):
        self.controller.move_to_position(self.waypoints[index])

    def go_to_next_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints) - 1:
            self.all_waypoints_reached = True
            return

        self.current_waypoint_index += 1
        self.go_to_waypoint(self.current_waypoint_index)