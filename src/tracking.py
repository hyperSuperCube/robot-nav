#!/usr/bin/env python3
import heapq
import math
import pickle
from enum import Enum, IntEnum


class Direction(IntEnum):
    """
    Discrete robot headings on the map grid.

    Headings are spaced by 45 degrees. Status list entry 0 is north, then
    entries proceed clockwise until entry 7 is northwest.
    """
    NORTH = 0
    NORTHEAST = 1
    EAST = 2
    SOUTHEAST = 3
    SOUTH = 4
    SOUTHWEST = 5
    WEST = 6
    NORTHWEST = 7


class StreetStatus(Enum):
    """
    Map state for one outgoing street direction at one intersection.

    UNKNOWN:
        The robot has not checked this direction yet.
    NONEXISTENT:
        The robot turned past this direction and did not see a street.
    UNEXPLORED:
        A street was detected, but the other end has not been visited yet.
    DEADEND:
        The street exists but ends without another intersection.
    CONNECTED:
        The street connects this intersection to another known intersection.
    """
    UNKNOWN = 0
    NONEXISTENT = 1
    UNEXPLORED = 2
    DEADEND = 3
    CONNECTED = 4


UNFINISHED_STREET_STATUSES = (
    StreetStatus.UNKNOWN,
    StreetStatus.UNEXPLORED,
)

HEADING_STEP_RADIANS = math.pi / 4.0

HEADING_VECTORS = {
    Direction.NORTH: (0, 1),
    Direction.NORTHEAST: (1, 1),
    Direction.EAST: (1, 0),
    Direction.SOUTHEAST: (1, -1),
    Direction.SOUTH: (0, -1),
    Direction.SOUTHWEST: (-1, -1),
    Direction.WEST: (-1, 0),
    Direction.NORTHWEST: (-1, 1),
}

STATUS_COLORS = {
    StreetStatus.UNKNOWN: "black",
    StreetStatus.NONEXISTENT: "lightgray",
    StreetStatus.UNEXPLORED: "blue",
    StreetStatus.DEADEND: "red",
    StreetStatus.CONNECTED: "green",
}


def normalize_heading(heading):
    """
    Convert any integer-like heading into a valid 'Direction'.

    Args:
        heading: A 'Direction' or integer heading index. Values outside the
            valid range wrap around the eight compass directions.

    Returns:
        A normalized 'Direction' value.
    """
    return Direction(int(heading) % len(Direction))


def calc_move(xold, yold, heading):
    """
    Compute the next grid coordinate after moving one edge forward.

    Args:
        xold: Current integer x coordinate.
        yold: Current integer y coordinate.
        heading: Direction of travel.

    Returns:
        Tuple '(xnew, ynew)' for the adjacent intersection in that direction.
    """
    dx, dy = HEADING_VECTORS[normalize_heading(heading)]
    return xold + dx, yold + dy


def direction_cost(heading):
    """
    Return the movement cost for one map direction.

    Cardinal streets cost 1. Diagonal streets cost sqrt(2).
    """
    dx, dy = HEADING_VECTORS[normalize_heading(heading)]
    return math.hypot(dx, dy)


def calc_turn(oldheading, turncount):
    """
    Compute a new heading after a discrete turn.

    Args:
        oldheading: Starting heading.
        turncount: Number of 45-degree heading steps to turn. Positive values
            turn left; negative values turn right.

    Returns:
        The new normalized 'Direction'.
    """
    return normalize_heading(int(oldheading) - int(turncount))


def calc_uturn(oldheading):
    """
    Compute the heading after a 180-degree turn.

    Args:
        oldheading: Starting heading.

    Returns:
        The opposite 'Direction'.
    """
    return calc_turn(oldheading, 4)


class Intersection:
    """
    Store map information for one grid intersection.

    'status[direction]' stores the map state for each direction. Entry 0 is
    north, then entries proceed clockwise until entry 7 is northwest.
    """
    def __init__(self, x, y):
        """
        Create one intersection node.

        Args:
            x: Integer x coordinate on the map grid.
            y: Integer y coordinate on the map grid.

        Returns:
            None.
        """
        self.x = int(x)
        self.y = int(y)
        self.status = [StreetStatus.UNKNOWN for _ in Direction]
        self.cost_to_goal = math.inf
        self.direction_to_goal = None

    def mark_nonexistent(self, direction):
        """
        Mark one direction as having no street unless it is locked.

        Connected streets, confirmed dead ends, and confirmed nonexistent
        streets are preserved.

        Args:
            direction: Direction to mark.

        Returns:
            None.
        """
        index = int(normalize_heading(direction))
        self.status[index] = StreetStatus.NONEXISTENT

    def mark_unknown_nonexistent(self, direction):
        """
        Mark an unknown direction nonexistent without erasing known streets.

        Args:
            direction: Direction to mark.

        Returns:
            None.
        """
        index = int(normalize_heading(direction))
        if self.status[index] is StreetStatus.UNKNOWN:
            self.status[index] = StreetStatus.NONEXISTENT

    def mark_adjacent_nonexistent(self, direction):
        """
        Apply the optional no-45-degree-adjacent-streets assumption.

        This is deliberately conservative: it only clears UNKNOWN directions,
        so a detected Y branch cannot be erased by a neighboring street.

        Args:
            direction: Direction where a real street exists.

        Returns:
            None.
        """

        self.mark_unknown_nonexistent(calc_turn(direction, -1))
        self.mark_unknown_nonexistent(calc_turn(direction, 1))

    def mark_unexplored(self, direction, assume_adjacent=False):
        """
        Mark one direction as a detected but unvisited street.

        Already known directions are preserved.

        Args:
            direction: Direction to mark.
            assume_adjacent: If True, also apply the optional 45-degree
                adjacency assumption around this newly detected street.

        Returns:
            None.
        """
        index = int(normalize_heading(direction))
        self.status[index] = StreetStatus.UNEXPLORED
        if assume_adjacent:
            self.mark_adjacent_nonexistent(direction)

    def mark_dead_end(self, direction):
        """
        Mark one direction as a street that ends without another intersection.

        Args:
            direction: Direction of the dead-end street from this intersection.

        Returns:
            None.
        """
        index = int(normalize_heading(direction))
        self.status[index] = StreetStatus.DEADEND

    def connect(self, direction, other):
        """
        Connect this intersection to another intersection.

        The connection is bidirectional in status: this intersection records
        `CONNECTED` in `direction`, and `other` records `CONNECTED` in the
        opposite direction. The neighbor itself is derived from coordinates.

        Args:
            direction: Direction from this intersection to 'other'.
            other: The connected 'Intersection' object.

        Returns:
            None.
        """
        direction = normalize_heading(direction)
        opposite = calc_uturn(direction)

        direction_index = int(direction)
        opposite_index = int(opposite)

        self.status[direction_index] = StreetStatus.CONNECTED

        other.status[opposite_index] = StreetStatus.CONNECTED

    def unfinished_directions(self):
        """
        Return directions that still need exploration information.

        Args:
            None.

        Returns:
            List of `Direction` values whose status is UNKNOWN or UNEXPLORED.
        """
        return [
            direction
            for direction in Direction
            if self.status[int(direction)] in UNFINISHED_STREET_STATUSES
        ]

    def is_unfinished(self):
        """
        Return whether this intersection still has unknown or unexplored streets.

        Args:
            None.

        Returns:
            Boolean.
        """
        return bool(self.unfinished_directions())


class Map:
    """
    Store intersections and provide map-level operations.
    """
    def __init__(self):
        self.intersections = {}
        self.goal = None

    def get_intersection(self, x, y):
        """
        Fetch or create an intersection at a grid coordinate.

        Args:
            x: Integer x coordinate.
            y: Integer y coordinate.

        Returns:
            The existing or newly created 'Intersection'.
        """
        key = (int(x), int(y))
        if key not in self.intersections:
            self.intersections[key] = Intersection(*key)
        return self.intersections[key]

    def set_street(self, x, y, heading, state):
        """
        Update one street state from the map API.

        Args:
            x: Intersection x coordinate.
            y: Intersection y coordinate.
            heading: Street direction to update.
            state: New `StreetStatus`.

        Returns:
            The final street status.
        """
        intersection = self.get_intersection(x, y)
        heading = normalize_heading(heading)
        state = StreetStatus(state)

        if state is StreetStatus.NONEXISTENT:
            intersection.mark_nonexistent(heading)
        elif state is StreetStatus.UNEXPLORED:
            intersection.mark_unexplored(heading)
        elif state is StreetStatus.DEADEND:
            intersection.mark_dead_end(heading)
        elif state is StreetStatus.CONNECTED:
            xnext, ynext = calc_move(x, y, heading)
            other = self.get_intersection(xnext, ynext)
            intersection.connect(heading, other)
        return intersection.status[int(heading)]

    def show(self, visualizer=None):
        """
        Visualize the map without a robot pose.

        Args:
            visualizer: Optional `MapVisualizer`.

        Returns:
            None.
        """
        if visualizer is None:
            visualizer = MapVisualizer()
        visualizer.show_map(self)

    def show_with_robot(self, xbot, ybot, hbot, visualizer=None):
        """
        Visualize the map with a robot pose overlay.

        Args:
            xbot: Robot x coordinate.
            ybot: Robot y coordinate.
            hbot: Robot heading.
            visualizer: Optional `MapVisualizer`.

        Returns:
            None.
        """
        if visualizer is None:
            visualizer = MapVisualizer()
        visualizer.show_map(self, xbot, ybot, hbot)

    def reset_planning(self):
        """
        Clear Dijkstra cost and direction data from every intersection.

        Args:
            None.

        Returns:
            None.
        """
        self.goal = None
        for intersection in self.intersections.values():
            intersection.cost_to_goal = math.inf
            intersection.direction_to_goal = None

    def unfinished_intersections(self):
        """
        Return known intersections with UNKNOWN or UNEXPLORED outgoing streets.

        Args:
            None.

        Returns:
            List of unfinished `Intersection` objects.
        """
        return [
            intersection
            for intersection in self.intersections.values()
            if intersection.is_unfinished()
        ]

    def save_map(self, filename):
        """
        Save the known map to a pickle file.

        Args:
            filename: Destination pickle filename.

        Returns:
            None.
        """
        with open(filename, "wb") as file:
            pickle.dump(self, file)

    def load_map(self, filename):
        """
        Load a known map from a pickle file.

        Args:
            filename: Source pickle filename.

        Returns:
            None.
        """
        with open(filename, "rb") as file:
            data = pickle.load(file)

        self.intersections = data.intersections
        self.reset_planning()

    def _dijkstra_from_goals(self, goals, visualizer=None):
        """
        Compute shortest directions from every reachable node to any goal.

        Args:
            goals: Iterable of zero-cost goal `Intersection` objects.
            visualizer: Optional `MapVisualizer` to update while planning.

        Returns:
            None.
        """
        self.reset_planning()

        on_deck = []
        for intersection in goals:
            intersection.cost_to_goal = 0
            heapq.heappush(on_deck, (0, intersection.x, intersection.y, intersection))

        while on_deck:
            cost, _, _, intersection = heapq.heappop(on_deck)
            if cost > intersection.cost_to_goal:
                continue

            if visualizer is not None:
                visualizer.show(self)

            for direction in Direction:
                if intersection.status[int(direction)] is not StreetStatus.CONNECTED:
                    continue

                xnext, ynext = calc_move(intersection.x, intersection.y, direction)
                neighbor = self.intersections[(xnext, ynext)]
                direction_from_neighbor = calc_uturn(direction)
                next_cost = cost + direction_cost(direction)
                if next_cost < neighbor.cost_to_goal:
                    neighbor.cost_to_goal = next_cost
                    neighbor.direction_to_goal = direction_from_neighbor
                    heapq.heappush(
                        on_deck,
                        (next_cost, neighbor.x, neighbor.y, neighbor),
                    )

        if visualizer is not None:
            visualizer.show(self)

    def dijkstra(self, xgoal, ygoal, visualizer=None):
        """
        Compute shortest directions from every reachable node to a goal.

        The known map uses only streets with `StreetStatus.CONNECTED`.
        Cardinal streets cost 1 and diagonal streets cost sqrt(2). Each
        intersection stores its `cost_to_goal` and `direction_to_goal`.

        Args:
            xgoal: Goal x coordinate.
            ygoal: Goal y coordinate.
            visualizer: Optional `MapVisualizer` to update while planning.

        Returns:
            `True` if the goal exists in the map, otherwise `False`.
        """
        goal_key = (int(xgoal), int(ygoal))
        self._dijkstra_from_goals([self.intersections[goal_key]], visualizer)
        self.goal = goal_key
        return True

    def dijkstra_to_nearest_unfinished(self, exclude_current=True, visualizer=None):
        """
        Compute shortest directions to the nearest unfinished intersection.

        All unfinished intersections are seeded as zero-cost goals, so the
        current pose receives a direction toward the closest unfinished node
        over known CONNECTED streets.

        Args:
            exclude_current: If True, ignore the robot's current intersection
                as a goal. Local exploration should handle it first.
            visualizer: Optional `MapVisualizer` to update while planning.

        Returns:
            True when a reachable unfinished intersection was found.
        """
        current_key = (self.x, self.y) if exclude_current else None

        goals = [
            intersection
            for intersection in self.unfinished_intersections()
            if (intersection.x, intersection.y) != current_key
        ]

        self._dijkstra_from_goals(goals)

        if not math.isfinite(self.current_intersection.cost_to_goal):
            return False

        intersection = self.current_intersection
        while True:
            key = (intersection.x, intersection.y)
            if intersection.is_unfinished() and key != current_key:
                return self.dijkstra(intersection.x, intersection.y, visualizer)

            direction = intersection.direction_to_goal
            xnext, ynext = calc_move(intersection.x, intersection.y, direction)
            intersection = self.intersections[(xnext, ynext)]


class RobotTracker(Map):
    """
    Track the robot pose and incrementally build the street map.

    The robot starts on a street, facing north by default. The first
    intersection it reaches becomes the map origin.
    """
    def __init__(self, heading=Direction.NORTH):
        """
        Create a tracker with an unknown starting position.

        Args:
            heading: Initial robot heading before the first intersection is
                acquired. Defaults to 'Direction.NORTH'.

        Returns:
            None.
        """
        super().__init__()
        self.x = 0
        self.y = 0
        self.heading = normalize_heading(heading)
        self.current_intersection = None
        self.find_start = False

    @property
    def pose(self):
        """
        Return the current tracked robot pose.

        Args:
            None.

        Returns:
            Tuple '(x, y, heading)'.
        """
        return self.x, self.y, self.heading

    def acquire_starting_intersection(self, x, y, heading):
        """
        Use the first detected intersection as the starting map pose.

        The street the robot used to enter this intersection is known to exist,
        but we do not yet know whether it connects to another intersection.

        Args:
            x: Starting intersection x coordinate.
            y: Starting intersection y coordinate.
            heading: Robot heading at the starting intersection.

        Returns:
            The starting 'Intersection'.
        """
        self.heading = normalize_heading(heading)
        self.x = int(x)
        self.y = int(y)
        self.current_intersection = self.get_intersection(self.x, self.y)
        self.find_start = True
        self.current_intersection.mark_unexplored(
            calc_uturn(self.heading),
            assume_adjacent=True,
        )
        return self.current_intersection

    def set_pose(self, x, y, heading):
        """
        Set the robot pose on a known intersection.

        Args:
            x: Integer x coordinate.
            y: Integer y coordinate.
            heading: Current robot heading.

        Returns:
            The current `Intersection`.

        """
        key = (int(x), int(y))
        self.x, self.y = key
        self.heading = normalize_heading(heading)
        self.current_intersection = self.intersections[key]
        self.find_start = True
        return self.current_intersection

    def load_map(self, filename):
        """
        Load a known map from a pickle file and clear robot pose.

        Args:
            filename: Source pickle filename.

        Returns:
            None.
        """
        super().load_map(filename)
        self.x = 0
        self.y = 0
        self.current_intersection = None
        self.find_start = False

    def clear_goal(self):
        """
        Remove the active goal and Dijkstra tree.

        Args:
            None.

        Returns:
            None.
        """
        self.reset_planning()

    def goal_reached(self):
        """
        Return whether the current pose is at the active goal.

        Args:
            None.

        Returns:
            Boolean.
        """
        return self.goal is not None and (self.x, self.y) == self.goal

    def current_plan_direction(self):
        """
        Return the next heading along the Dijkstra tree from the current pose.

        Args:
            None.

        Returns:
            A `Direction`, or `None` if no active path is available.
        """
        return self.current_intersection.direction_to_goal

    def planned_path_from_current(self):
        """
        Return the planned coordinate path from the current pose to the goal.

        Args:
            None.

        Returns:
            List of `(x, y)` coordinates. Empty if no path is available.
        """
        path = []
        intersection = self.current_intersection

        while True:
            key = (intersection.x, intersection.y)
            path.append(key)

            if key == self.goal:
                return path

            direction = intersection.direction_to_goal
            xnext, ynext = calc_move(intersection.x, intersection.y, direction)
            intersection = self.intersections[(xnext, ynext)]

    def record_turn(self, turn_estimate):
        """
        Update heading after the robot turns at the current intersection.

        Directions passed during the turn are marked nonexistent. The final
        heading is marked unexplored only when it is still unknown.

        Args:
            turn_estimate: A 'TurnEstimate' object from the behavior layer.

        Returns:
            Integer number of 45-degree heading steps used for the turn.
        """
        old_heading = self.heading
        turncount = int(round(turn_estimate.angle / HEADING_STEP_RADIANS))

        left_or_right = 1 if turncount > 0 else -1
        turn_steps = abs(turncount)

        if self.find_start:
            for offset in range(1, turn_steps):
                passed = calc_turn(old_heading, left_or_right * offset)
                self.current_intersection.mark_unknown_nonexistent(passed)

        self.heading = calc_turn(old_heading, left_or_right * turn_steps)
        if self.find_start:
            heading_index = int(self.heading)
            if (
                self.current_intersection.status[heading_index]
                is StreetStatus.UNKNOWN
            ):
                self.current_intersection.mark_unexplored(
                    self.heading,
                    assume_adjacent=True,
                )
        return left_or_right * turn_steps

    def record_move(self):
        """
        Update position after moving from one intersection to the next.

        If the street in the current heading is already connected, the tracker
        moves to that known intersection. Otherwise it creates the adjacent
        intersection and connects the two endpoints.

        Args:
            None.

        Returns:
            The intersection reached by the move.

        """
        old_intersection = self.current_intersection
        old_heading = self.heading

        xnew, ynew = calc_move(self.x, self.y, old_heading)
        if old_intersection.status[int(old_heading)] is StreetStatus.CONNECTED:
            new_intersection = self.intersections[(xnew, ynew)]
        else:
            new_intersection = self.get_intersection(xnew, ynew)
            old_intersection.connect(old_heading, new_intersection)

        self.x = new_intersection.x
        self.y = new_intersection.y
        self.current_intersection = new_intersection
        return new_intersection

    def record_dead_end_uturn(self):
        """
        Mark a dead end and update heading for the return trip.

        If the robot was moving along a known connected edge when it reached an
        end, the dead end is assigned to the known intersection ahead. This
        handles cases where the robot physically passes a known intersection
        before detecting the end.

        Args:
            None.

        Returns:
            The 'Intersection' whose outgoing street was marked as a dead end,
            or 'None' if the start has not been acquired yet.
        """
        old_heading = self.heading
        return_heading = calc_uturn(old_heading)

        if self.current_intersection.status[int(old_heading)] is StreetStatus.CONNECTED:
            xdead, ydead = calc_move(self.x, self.y, old_heading)
            dead_end_intersection = self.intersections[(xdead, ydead)]
        else:
            dead_end_intersection = self.current_intersection

        dead_end_intersection.mark_dead_end(old_heading)
        self.heading = return_heading
        self.current_intersection = dead_end_intersection
        self.x = dead_end_intersection.x
        self.y = dead_end_intersection.y
        return dead_end_intersection


class MapVisualizer:
    """
    Live Matplotlib visualization for the tracker.
    """
    def __init__(self, enabled=True):
        """
        Create a live Matplotlib map visualizer.

        Args:
            enabled: If False, visualization calls do nothing.

        Returns:
            None.
        """
        self.enabled = enabled
        self.plt = None
        self.figure = None
        self.failed = False

        if self.enabled:
            try:
                import matplotlib.pyplot as plt
                self.plt = plt
                self.plt.ion()
                self.figure = self.plt.figure("Robot map")
            except Exception as exc:
                self.failed = True
                print("Tracking visualization disabled: %s" % exc)

    def show(self, tracker):
        """
        Draw the current tracker map and robot pose.

        Args:
            tracker: 'RobotTracker' whose intersections, street statuses, and
                pose should be visualized.

        Returns:
            None.
        """
        self.show_map(tracker, tracker.x, tracker.y, tracker.heading)

    def show_map(self, map_object, xbot=None, ybot=None, hbot=None):
        """
        Draw a map, optionally with a robot pose.

        Args:
            map_object: `Map` or `RobotTracker` to draw.
            xbot: Optional robot x coordinate.
            ybot: Optional robot y coordinate.
            hbot: Optional robot heading.

        Returns:
            None.
        """
        if not self.enabled or self.failed:
            return

        plt = self.plt
        plt.figure(self.figure.number)
        plt.clf()
        ax = plt.gca()

        x_values = []
        y_values = []
        if xbot is not None and ybot is not None:
            x_values.append(xbot)
            y_values.append(ybot)

        for x, y in map_object.intersections:
            x_values.append(x)
            y_values.append(y)

        if not x_values:
            x_values.append(0)
            y_values.append(0)

        xmin = math.floor(min(x_values)) - 2
        xmax = math.ceil(max(x_values)) + 2
        ymin = math.floor(min(y_values)) - 2
        ymax = math.ceil(max(y_values)) + 2

        ax.set_xlim(xmin - 0.5, xmax + 0.5)
        ax.set_ylim(ymin - 0.5, ymax + 0.5)
        ax.set_aspect("equal")
        ax.grid(True)

        for x in range(xmin, xmax + 1):
            for y in range(ymin, ymax + 1):
                ax.plot(x, y, color="lightgray", marker="o", markersize=5)

        for intersection in map_object.intersections.values():
            for direction in Direction:
                status = intersection.status[int(direction)]
                color = STATUS_COLORS[status]
                dx, dy = HEADING_VECTORS[direction]
                ax.plot(
                    [intersection.x, intersection.x + 0.5 * dx],
                    [intersection.y, intersection.y + 0.5 * dy],
                    color=color,
                    linewidth=2,
                )

        for intersection in map_object.intersections.values():
            ax.plot(
                intersection.x,
                intersection.y,
                color="black",
                marker="o",
                markersize=6,
            )
            self._draw_plan_direction(ax, intersection)

        if xbot is not None and ybot is not None and hbot is not None:
            hbot = normalize_heading(hbot)
            self._draw_robot(ax, xbot, ybot, hbot)
            title = "Robot pose: x=%d, y=%d, heading=%s" % (
                xbot,
                ybot,
                hbot.name,
            )
        else:
            title = "Robot map"

        if map_object.goal is not None:
            title += ", goal=(%d, %d)" % map_object.goal
        ax.set_title(title)
        plt.pause(0.001)

    def _draw_plan_direction(self, ax, intersection):
        """
        Draw one Dijkstra direction arrow and cost label.

        Args:
            ax: Matplotlib axes object to draw on.
            intersection: `Intersection` to annotate.

        Returns:
            None.
        """
        cost = getattr(intersection, "cost_to_goal", math.inf)
        if not math.isfinite(cost):
            return

        ax.text(
            intersection.x + 0.08,
            intersection.y + 0.08,
            "%.2f" % cost,
            color="orange",
            fontsize=8,
        )

        direction = getattr(intersection, "direction_to_goal", None)
        if direction is None:
            return

        dx, dy = HEADING_VECTORS[normalize_heading(direction)]
        ax.arrow(
            intersection.x,
            intersection.y,
            0.35 * dx,
            0.35 * dy,
            width=0.025,
            head_width=0.14,
            head_length=0.08,
            color="orange",
            length_includes_head=True,
        )

    def _draw_robot(self, ax, x, y, heading):
        """
        Draw the robot arrow on the Matplotlib axes.

        Args:
            ax: Matplotlib axes object to draw on.
            x: Robot x coordinate.
            y: Robot y coordinate.
            heading: Current robot heading.

        Returns:
            None.
        """
        dx, dy = HEADING_VECTORS[normalize_heading(heading)]
        length = math.hypot(dx, dy)
        ux = dx / length
        uy = dy / length

        arrow_length = 0.5
        xbase = x - 0.5 * arrow_length * ux
        ybase = y - 0.5 * arrow_length * uy
        xtip = x + 0.5 * arrow_length * ux
        ytip = y + 0.5 * arrow_length * uy

        ax.arrow(
            xbase,
            ybase,
            xtip - xbase,
            ytip - ybase,
            width=0.08,
            head_width=0.25,
            head_length=0.12,
            color="magenta",
            length_includes_head=True,
        )
