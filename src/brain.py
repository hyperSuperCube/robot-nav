#!/usr/bin/env python3
import math
from enum import Enum, auto

from behaviors import (
    BasicBehaviors,
    LineFollowerResult,
)
from constants import param
from tracking import Direction, MapVisualizer, RobotTracker, StreetStatus


class OperatorCommand(Enum):
    STRAIGHT = auto()
    LEFT = auto()
    RIGHT = auto()
    SAVE = auto()
    LOAD = auto()
    GOTO = auto()
    EXPLORE = auto()
    QUIT = auto()


class SimpleBrain:
    """
    Operator-facing navigation logic.

    The brain talks only to the behavior layer. It does not construct or
    directly control motors, sensors, or GPIO.
    """
    def __init__(self, behavior, tracker=None, visualizer=None):
        self.behavior = behavior
        self.tracker = tracker if tracker is not None else RobotTracker()
        self.visualizer = visualizer if visualizer is not None else MapVisualizer()

    def _show_tracker(self):
        """
        Visualize and print the current tracked pose.
        """
        self.visualizer.show(self.tracker)
        x, y, heading = self.tracker.pose
        print("Tracked pose: x=%d, y=%d, heading=%s." % (x, y, heading.name))

    def _print_turn_estimate(self, turn_estimate):
        """
        Print the turn estimate in a compact debugging format.
        """
        message = (
            "Turn estimate: %.1f deg classified, %.3f s"
            % (turn_estimate.degrees, turn_estimate.elapsed)
        )
        if turn_estimate.gyro_angle is not None:
            message += ", %.1f deg gyro" % math.degrees(turn_estimate.gyro_angle)
        message += ", %.1f deg table" % math.degrees(turn_estimate.timed_angle)
        print(message)

    def _acquire_starting_intersection(self):
        """
        Follow the starting street until the first intersection is found.
        """
        if self.tracker.find_start:
            return

        print("Finding first intersection to set map origin.")
        while True:
            result = self.behavior.follow_line()

            if result is LineFollowerResult.INTERSECTION:
                print("First intersection detected.")
                print("Pulling through intersection.")
                self.behavior.pull_forward()
                print("Enter the map pose for this starting intersection.")
                x, y, heading = self._prompt_pose()
                self.tracker.acquire_starting_intersection(x, y, heading)
                return

            print("End of line before first intersection. Turning around.")
            turn_estimate = self.behavior.turn_left()
            self._print_turn_estimate(turn_estimate)
            self.tracker.record_turn(turn_estimate)

    def _prompt_navigation_command(self):
        """
        Ask the operator what to do after stopping.
        """
        command_aliases = {
            "": OperatorCommand.STRAIGHT,
            "s": OperatorCommand.STRAIGHT,
            "straight": OperatorCommand.STRAIGHT,
            "l": OperatorCommand.LEFT,
            "left": OperatorCommand.LEFT,
            "r": OperatorCommand.RIGHT,
            "right": OperatorCommand.RIGHT,
            "w": OperatorCommand.SAVE,
            "write": OperatorCommand.SAVE,
            "save": OperatorCommand.SAVE,
            "load": OperatorCommand.LOAD,
            "open": OperatorCommand.LOAD,
            "g": OperatorCommand.GOTO,
            "go": OperatorCommand.GOTO,
            "goto": OperatorCommand.GOTO,
            "e": OperatorCommand.EXPLORE,
            "explore": OperatorCommand.EXPLORE,
            "auto": OperatorCommand.EXPLORE,
            "q": OperatorCommand.QUIT,
            "quit": OperatorCommand.QUIT,
            "exit": OperatorCommand.QUIT,
        }

        while True:
            raw_command = input(
                "Command [s]traight/[l]eft/[r]ight/[e]xplore/[g]oto/[w]rite/load/[q]uit: "
            ).strip().lower()

            command = command_aliases.get(raw_command)
            if command is not None:
                return command
            else:
                print("Unknown command. Enter s, l, r, e, g, w, load, or q.")

    def _prompt_filename(self, action):
        """
        Ask for a pickle filename.
        """
        raw_filename = input(
            "%s map filename [%s]: " % (action, param.map_filename)
        ).strip()
        if raw_filename == "":
            return param.map_filename
        return raw_filename

    def _parse_heading(self, raw_heading):
        """
        Parse a heading from text.
        """
        aliases = {
            "n": Direction.NORTH,
            "north": Direction.NORTH,
            "nw": Direction.NORTHWEST,
            "northwest": Direction.NORTHWEST,
            "w": Direction.WEST,
            "west": Direction.WEST,
            "sw": Direction.SOUTHWEST,
            "southwest": Direction.SOUTHWEST,
            "s": Direction.SOUTH,
            "south": Direction.SOUTH,
            "se": Direction.SOUTHEAST,
            "southeast": Direction.SOUTHEAST,
            "e": Direction.EAST,
            "east": Direction.EAST,
            "ne": Direction.NORTHEAST,
            "northeast": Direction.NORTHEAST,
        }

        raw_heading = raw_heading.strip().lower()
        if raw_heading in aliases:
            return aliases[raw_heading]
        if raw_heading.upper() in Direction.__members__:
            return Direction[raw_heading.upper()]
        return Direction(int(raw_heading))

    def _prompt_pose(self):
        """
        Ask for the robot pose on the map.
        """
        while True:
            raw_pose = input("Robot pose x y heading: ").replace(",", " ")
            parts = raw_pose.split()
            if len(parts) != 3:
                print("Enter pose as: x y heading")
                continue

            try:
                return int(parts[0]), int(parts[1]), self._parse_heading(parts[2])
            except (TypeError, ValueError):
                print("Could not parse pose. Example: 0 0 north")

    def _prompt_goal(self):
        """
        Ask for a goal coordinate.
        """
        while True:
            raw_goal = input("Goal x y: ").replace(",", " ")
            parts = raw_goal.split()
            if len(parts) != 2:
                print("Enter goal as: x y")
                continue

            try:
                return int(parts[0]), int(parts[1])
            except ValueError:
                print("Could not parse goal. Example: 2 -1")

    def _execute_turn_command(self, command):
        """
        Execute a selected left or right operator command.
        """
        if command is OperatorCommand.LEFT:
            print("Turning left.")
            turn_estimate = self.behavior.turn_left()
        else:
            print("Turning right.")
            turn_estimate = self.behavior.turn_right()

        self._print_turn_estimate(turn_estimate)
        turncount = self.tracker.record_turn(turn_estimate)
        print(
            "Rounded turn count: %d, new heading: %s."
            % (turncount, self.tracker.heading.name)
        )

    def _execute_straight_command(self, automatic=False):
        """
        Execute a forward street-following command.
        """
        if self.tracker.find_start:
            intersection = self.tracker.current_intersection
            if intersection is None:
                print("No current intersection for driving.")
                self.tracker.clear_goal()
                return

            status = intersection.status[int(self.tracker.heading)]
            if automatic:
                allowed = status is StreetStatus.CONNECTED
            else:
                allowed = status in (
                    StreetStatus.CONNECTED,
                    StreetStatus.UNEXPLORED,
                    StreetStatus.UNKNOWN,
                )

            if not allowed:
                print(
                    "Cannot drive %s because map status is %s."
                    % (self.tracker.heading.name, status.name)
                )
                if automatic:
                    self.tracker.clear_goal()
                return

        print("Line following.")
        result = self.behavior.follow_line()
        self._handle_line_following_result(result)

    def _execute_save_command(self):
        """
        Save the current map to a pickle file.
        """
        filename = self._prompt_filename("Save")
        print("Saving the map to %s..." % filename)
        self.tracker.save_map(filename)

    def _execute_load_command(self):
        """
        Load a map from a pickle file and set the robot pose.
        """
        filename = self._prompt_filename("Load")
        print("Loading the map from %s..." % filename)
        try:
            self.tracker.load_map(filename)
        except OSError as exc:
            print("Could not load map: %s" % exc)
            return

        while True:
            x, y, heading = self._prompt_pose()
            try:
                self.tracker.set_pose(x, y, heading)
                break
            except ValueError as exc:
                print(exc)

        self._show_tracker()

    def _execute_goto_command(self):
        """
        Plan an autonomous path to a goal.
        """
        if not self.tracker.find_start:
            print("Set the robot pose before planning.")
            return

        xgoal, ygoal = self._prompt_goal()
        print("Planning path to (%d, %d)..." % (xgoal, ygoal))
        if not self.tracker.dijkstra(xgoal, ygoal, visualizer=self.visualizer):
            print("Goal is not in the loaded map.")
            return

        if self.tracker.goal_reached():
            print("Already at the goal.")
            self.tracker.clear_goal()
            return

        if self.tracker.current_plan_direction() is None:
            print("Goal is unreachable from the current pose.")
            self.tracker.clear_goal()
            return

        print("Planned path: %s" % (self.tracker.planned_path_from_current(),))

    def _choose_autonomous_command(self):
        """
        Choose the next action from the active Dijkstra tree.
        """
        if self.tracker.goal_reached():
            print("Goal reached.")
            self.tracker.clear_goal()
            return None

        target_heading = self.tracker.current_plan_direction()
        if target_heading is None:
            print("No path from current pose to goal.")
            self.tracker.clear_goal()
            return None

        if self.tracker.heading == target_heading:
            return OperatorCommand.STRAIGHT

        return self._turn_command_toward(target_heading)

    def _turn_command_toward(self, target_heading):
        left_steps = (
            int(self.tracker.heading) - int(target_heading)
        ) % len(Direction)
        right_steps = (
            int(target_heading) - int(self.tracker.heading)
        ) % len(Direction)
        if left_steps <= right_steps:
            return OperatorCommand.LEFT
        return OperatorCommand.RIGHT

    def _choose_local_exploration_command(self):
        intersection = self.tracker.current_intersection
        if intersection is None:
            return None

        current_status = intersection.status[int(self.tracker.heading)]
        if current_status is StreetStatus.UNEXPLORED:
            print("Exploration: driving current UNEXPLORED street.")
            return OperatorCommand.STRAIGHT

        for target_status in (StreetStatus.UNKNOWN, StreetStatus.UNEXPLORED):
            candidates = [
                direction
                for direction in Direction
                if (
                    direction != self.tracker.heading
                    and intersection.status[int(direction)] is target_status
                )
            ]
            if not candidates:
                continue

            target_heading = min(
                candidates,
                key=lambda direction: min(
                    (int(direction) - int(self.tracker.heading)) % len(Direction),
                    (int(self.tracker.heading) - int(direction)) % len(Direction),
                ),
            )
            print(
                "Exploration: turning toward %s %s street."
                % (target_heading.name, target_status.name)
            )
            return self._turn_command_toward(target_heading)

        if current_status is StreetStatus.UNKNOWN:
            print("Exploration: testing current UNKNOWN street.")
            return OperatorCommand.STRAIGHT

        return None

    def _choose_exploration_command(self):
        if self.tracker.goal is not None:
            command = self._choose_autonomous_command()
            if command is not None:
                return command

        command = self._choose_local_exploration_command()
        if command is not None:
            self.tracker.clear_goal()
            return command

        if self.tracker.dijkstra_to_nearest_unfinished(visualizer=self.visualizer):
            print(
                "Exploration: routing to unfinished intersection %s."
                % (self.tracker.goal,)
            )
            print("Planned path: %s" % (self.tracker.planned_path_from_current(),))
            return self._choose_autonomous_command()

        if self.tracker.unfinished_intersections():
            print("Exploration stopped: unfinished intersections are unreachable.")
        else:
            print("Exploration complete: no unfinished intersections remain.")
        self.tracker.clear_goal()
        return None

    def _execute_explore_command(self):
        if not self.tracker.find_start:
            self._acquire_starting_intersection()

        print("Autonomous exploration started. Press Ctrl+C to stop anytime.")
        while True:
            self.behavior.stop()
            self._show_tracker()
            command = self._choose_exploration_command()
            if command is None:
                self.behavior.stop()
                return

            if command in (OperatorCommand.LEFT, OperatorCommand.RIGHT):
                self._execute_turn_command(command)
            else:
                self._execute_straight_command(automatic=False)

    def _handle_line_following_result(self, result):
        """
        Handle the result after the operator chooses straight.
        """
        if result is LineFollowerResult.INTERSECTION:
            print("Intersection detected.")
            print("Pulling through intersection.")
            self.behavior.pull_forward()
            self.tracker.record_move()
        elif result is LineFollowerResult.END_OF_STREET:
            self.tracker.record_dead_end_uturn()
            self._return_from_end_of_line()

    def _return_from_end_of_line(self):
        """
        U-turn at an end-of-line and recover onto the known map.
        """
        self.behavior.stop()
        print("End of line reached. Making automatic U-turn.")
        turn_estimate = self.behavior.turn_left()
        self._print_turn_estimate(turn_estimate)

        while True:
            result = self.behavior.follow_line()
            if result is LineFollowerResult.INTERSECTION:
                print("Returned to previous intersection.")
                print("Pulling through intersection.")
                self.behavior.pull_forward()
                break
            else:
                print("End of line reached while returning. Turning left again.")
                turn_estimate = self.behavior.turn_left()
                self._print_turn_estimate(turn_estimate)

    def run(self):
        print("Robot brain is ready.")
        if input("Load a saved map? [y/N]: ").strip().lower() in ("y", "yes"):
            self._execute_load_command()
            if not self.tracker.find_start:
                self._acquire_starting_intersection()
        else:
            self._acquire_starting_intersection()

        while True:
            self.behavior.stop()
            self._show_tracker()
            if self.tracker.goal is None:
                print("Robot stopped. Waiting for operator command.")
                command = self._prompt_navigation_command()
            else:
                command = self._choose_autonomous_command()
                if command is None:
                    continue

            if command is OperatorCommand.QUIT:
                print("Stopping for operator.")
                self.behavior.brake()
                self.behavior.stop()
                return
            elif command is OperatorCommand.SAVE:
                self._execute_save_command()
                continue
            elif command is OperatorCommand.LOAD:
                self._execute_load_command()
                continue
            elif command is OperatorCommand.GOTO:
                self._execute_goto_command()
                continue
            elif command is OperatorCommand.EXPLORE:
                self._execute_explore_command()
                continue
            elif command in (OperatorCommand.LEFT, OperatorCommand.RIGHT):
                self._execute_turn_command(command)
                continue
            else:
                self._execute_straight_command(
                    automatic=self.tracker.goal is not None,
                )
                continue


def main():
    tracker = RobotTracker()
    visualizer = MapVisualizer()
    with BasicBehaviors() as behavior:
        brain = SimpleBrain(behavior, tracker, visualizer)
        brain.run()


if __name__ == "__main__":
    main()
