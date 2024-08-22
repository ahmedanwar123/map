#!/usr/bin/env python3
"""
Task Description:

This script uses Pygame to display a minefield map, integrated with ROS for real-time updates.

- **Map Visualization**: Shows a grid-based map with tiles for empty spaces, surface mines, and buried mines.
- **ROS Integration**: Subscribes to topics for robot pose and mine detection, updating the map accordingly.
- **Coordinate System**: The map's origin (0,0) is at the bottom-left, with Y-axis increasing upwards.
- **Dynamic Updates**: Mines are placed based on the robot's position and orientation, displayed in two tables (surface and buried).

The script initializes Pygame, sets up ROS subscribers, and updates the map until the application exits.

*****************IMPORTANT: change the pkg name to 'map'**************************
"""

import time
import os
import pygame
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import math
import const
from const import MINE_TYPE

# Initialize Pygame
pygame.init()
pygame.display.set_caption("Minefield Map")

# Set up font
FONT = pygame.font.Font(None, const.FONT_SIZE)

orientation_offset: "float | None" = None
pose_x_offset: "float | None" = None
pose_y_offset: "float | None" = None

# Define the initial map (0 for empty space, 1 for surface mine, 2 for buried mine)
map_data = [[MINE_TYPE.NO_MINE] * const.MAP_WIDTH for _ in range(const.MAP_HEIGHT)]
first_detection: bool = True

# Robot's current position and orientation
robot_x, robot_y, robot_theta = 0, 0, 0  # Initialize with default values
camera_detected: "bool | None" = None
metal_detected: "bool | None" = None


# Active corner and orientation
active_corner = None
active_orientation = None
selection_complete: bool = False  # New variable to track if selection is complete


# Function to calculate mine position based on robot's position and orientation
def calculate_mine_position(rel_x: int, rel_y: int, theta: float) -> "tuple[int, int]":
    theta_rad = math.radians(theta)

    # Calculate absolute mine position based on robot's position and orientation
    abs_x = robot_x + rel_x * math.cos(theta_rad) - rel_y * math.sin(theta_rad)
    abs_y = robot_y + rel_x * math.sin(theta_rad) + rel_y * math.cos(theta_rad)

    # Convert to grid coordinates (assuming the robot's coordinates are also in grid units)
    grid_x = int(round(abs_x))
    grid_y = int(round(abs_y))

    return grid_x, grid_y


# ROS callback for robot position and orientation
def robot_pose_callback(data: Pose2D) -> None:
    global robot_x, robot_y, robot_theta
    try:
        if orientation_offset is not None:
            robot_theta = (
                data.theta + orientation_offset
            )  # The theta value is already in radians
            if (pose_x_offset, pose_y_offset) == const.CORNER_POSITIONS["Bottom-left"]:
                if orientation_offset == 0:
                    robot_x = data.x
                    robot_y = data.y
                elif orientation_offset == 90:
                    robot_x = data.y
                    robot_y = data.x
            elif (pose_x_offset, pose_y_offset) == const.CORNER_POSITIONS[
                "Bottom-right"
            ]:
                if orientation_offset == 90:
                    robot_x = const.MAP_WIDTH - data.y - 1
                    robot_y = data.x
                elif orientation_offset == 180:
                    robot_x = const.MAP_WIDTH - data.x - 1
                    robot_y = data.y
            elif (pose_x_offset, pose_y_offset) == const.CORNER_POSITIONS["Top-left"]:
                if orientation_offset == 0:
                    robot_x = data.x
                    robot_y = const.MAP_HEIGHT - data.y - 1
                elif orientation_offset == -90:
                    robot_x = data.y
                    robot_y = const.MAP_HEIGHT - data.x - 1
            elif (pose_x_offset, pose_y_offset) == const.CORNER_POSITIONS["Top-right"]:
                if orientation_offset == 180:
                    robot_x = const.MAP_WIDTH - data.x - 1
                    robot_y = const.MAP_HEIGHT - data.y - 1
                elif orientation_offset == -90:
                    robot_x = const.MAP_HEIGHT - data.y - 1
                    robot_y = const.MAP_HEIGHT - data.x - 1
            else:
                rospy.loginfo("orientation_offset is not received")

        rospy.loginfo(
            f"Robot position: ({robot_x}, {robot_y}), Orientation: {robot_theta:.2f} Degrees"
        )
    except Exception as e:
        rospy.logerr(f"Exception in robot_pose_callback: {e}")


# ROS callback for camera detection (Bool)
def camera_detection_callback(data: Bool) -> None:
    global camera_detected
    camera_detected = data.data  # True if a mine is detected by the camera
    rospy.loginfo(
        f"Camera detection: {'Mine detected' if camera_detected else 'No mine detected'}"
    )


# ROS callback for metal detector detection (Bool)
def metal_detector_callback(data: Bool) -> None:
    global metal_detected
    metal_detected = data.data  # True if a metal mine is detected, False otherwise
    rospy.loginfo(
        f"Metal detector detection: {'Mine detected' if metal_detected is True else 'No mine detected'}"
    )


def mine_detection_callback() -> None:
    global camera_detected, metal_detected, first_detection, map_data

    if not selection_complete:
        rospy.loginfo("Selection of corner and orientation not complete.")
        return

    if metal_detected:
        mine_type = "surface" if camera_detected else "buried"
    else:
        mine_type = "no mine"

    if first_detection:
        # Handle the glitch for the first detection by writing "no mine"
        rel_x, rel_y = 1, 0  # Assume the glitch happens 1 unit in front
        glitch_x, glitch_y = calculate_mine_position(rel_x, rel_y, robot_theta)

        if 0 <= glitch_x < const.MAP_WIDTH and 0 <= glitch_y < const.MAP_HEIGHT:
            map_data[glitch_y][glitch_x] = MINE_TYPE.NO_MINE

        first_detection = False  # Disable the glitch after it occurs once
        rospy.loginfo("Handled the first detection glitch.")
        return

    if mine_type != "no mine":
        # Assuming the mine is always 1 unit in front of the robot
        rel_x, rel_y = 1, 0  # Relative position of the mine (1 unit in front)
        mine_x, mine_y = calculate_mine_position(rel_x, rel_y, robot_theta)

        if 0 <= mine_x < const.MAP_WIDTH and 0 <= mine_y < const.MAP_HEIGHT:
            if mine_type == "surface":
                map_data[mine_y][mine_x] = MINE_TYPE.SURFACE
            elif mine_type == "buried":
                map_data[mine_y][mine_x] = MINE_TYPE.BURIED
        else:
            rospy.logwarn(f"Mine position out of bounds: ({mine_x}, {mine_y})")
    else:
        rospy.loginfo("No mine detected.")


def draw_map(screen: pygame.Surface) -> None:
    for y, row in enumerate(map_data):
        for x, tile in enumerate(row):
            if tile == MINE_TYPE.SURFACE:
                screen.blit(
                    const.SURFACE_MINE_IMG,
                    (x * const.TILE_SIZE, (const.MAP_HEIGHT - y - 1) * const.TILE_SIZE),
                )
            elif tile == MINE_TYPE.BURIED:
                screen.blit(
                    const.BURIED_MINE_IMG,
                    (x * const.TILE_SIZE, (const.MAP_HEIGHT - y - 1) * const.TILE_SIZE),
                )
            else:
                screen.blit(
                    const.EMPTY_IMG,
                    (x * const.TILE_SIZE, (const.MAP_HEIGHT - y - 1) * const.TILE_SIZE),
                )
            pygame.draw.rect(
                screen,
                (0, 0, 0),
                (
                    x * const.TILE_SIZE,
                    (const.MAP_HEIGHT - y - 1) * const.TILE_SIZE,
                    const.TILE_SIZE,
                    const.TILE_SIZE,
                ),
                1,
            )
            coord_text = FONT.render(f"{chr(65 + x)},{y + 1}", True, (0, 0, 0))
            screen.blit(
                coord_text,
                (
                    x * const.TILE_SIZE + 2,
                    (const.MAP_HEIGHT - y - 1) * const.TILE_SIZE + 2,
                ),
            )
    # Calculate robot image position
    robot_screen_x = int(robot_x * const.TILE_SIZE)
    robot_screen_y = int(
        (const.MAP_HEIGHT - robot_y - 1) * const.TILE_SIZE
    )  # Convert map coordinates to screen coordinates

    # Draw the robot image
    screen.blit(const.ROBOT_IMG, (robot_screen_x, robot_screen_y))

    # Draw the blue circle for the robot
    # robot_screen_x = int(robot_x * TILE_SIZE)
    # robot_screen_y = int((MAP_HEIGHT - robot_y - 1) * TILE_SIZE)
    # pygame.draw.circle(screen, (0, 0, 255), (robot_screen_x + TILE_SIZE // 2, robot_screen_y + TILE_SIZE // 2), TILE_SIZE // 4)  # Blue circle with radius half of TILE_SIZE

    # Draw the arrow to indicate the robot's orientation
    arrow_length = (const.TILE_SIZE // 2) + 20
    arrow_x = (
        robot_screen_x
        + const.TILE_SIZE // 2
        + arrow_length * math.cos(math.radians(robot_theta))
    )
    arrow_y = (
        robot_screen_y
        + const.TILE_SIZE // 2
        - arrow_length * math.sin(math.radians(robot_theta))
    )
    pygame.draw.line(
        screen,
        (255, 0, 0),
        (robot_screen_x + const.TILE_SIZE // 2, robot_screen_y + const.TILE_SIZE // 2),
        (arrow_x, arrow_y),
        12,
    )


# Function to draw the corner and orientation cells


def draw_corner_orientation_cells(screen: pygame.Surface) -> None:
    global active_corner, active_orientation, selection_complete

    if selection_complete:
        return  # No drawing if selection is complete

    if active_corner:
        # Draw only the active corner cell
        corner_x, corner_y = const.CORNER_POSITIONS[active_corner]
        pygame.draw.rect(
            screen,
            const.SELECTED_COLOR,
            (
                corner_x * const.TILE_SIZE,
                (const.MAP_HEIGHT - corner_y - 1) * const.TILE_SIZE,
                const.TILE_SIZE,
                const.TILE_SIZE,
            ),
        )

        # Draw the adjacent cells for orientation
        for i, (ox, oy) in enumerate(const.ORIENTATION_OFFSETS[active_corner]):
            orientation_x = corner_x + ox
            orientation_y = corner_y + oy
            if (
                0 <= orientation_x < const.MAP_WIDTH
                and 0 <= orientation_y < const.MAP_HEIGHT
            ):
                color = (
                    const.SELECTED_COLOR
                    if active_orientation == (active_corner, i)
                    else const.ORIENTATION_COLOR
                )
                pygame.draw.rect(
                    screen,
                    color,
                    (
                        orientation_x * const.TILE_SIZE,
                        (const.MAP_HEIGHT - orientation_y - 1) * const.TILE_SIZE,
                        const.TILE_SIZE,
                        const.TILE_SIZE,
                    ),
                )

    # When a corner and orientation are selected, do not draw any corners or ORIENTATIONS
    elif not active_corner:
        # Draw all corners initially
        for corner, (cx, cy) in const.CORNER_POSITIONS.items():
            pygame.draw.rect(
                screen,
                const.CORNER_COLOR,
                (
                    cx * const.TILE_SIZE,
                    (const.MAP_HEIGHT - cy - 1) * const.TILE_SIZE,
                    const.TILE_SIZE,
                    const.TILE_SIZE,
                ),
            )

        # Draw orientation cells around all corners
        for corner, (cx, cy) in const.CORNER_POSITIONS.items():
            for i, (ox, oy) in enumerate(const.ORIENTATION_OFFSETS[corner]):
                orientation_x = cx + ox
                orientation_y = cy + oy
                if (
                    0 <= orientation_x < const.MAP_WIDTH
                    and 0 <= orientation_y < const.MAP_HEIGHT
                ):
                    pygame.draw.rect(
                        screen,
                        const.ORIENTATION_COLOR,
                        (
                            orientation_x * const.TILE_SIZE,
                            (const.MAP_HEIGHT - orientation_y - 1) * const.TILE_SIZE,
                            const.TILE_SIZE,
                            const.TILE_SIZE,
                        ),
                    )


# Function to handle mouse click events
def handle_mouse_click(pos: "tuple[int, int]") -> None:
    global \
        robot_x, \
        robot_y, \
        robot_theta, \
        active_corner, \
        active_orientation, \
        selection_complete, \
        orientation_offset, \
        pose_x_offset, \
        pose_y_offset

    if selection_complete:
        return  # Ignore clicks if selection is complete

    mouse_x, mouse_y = pos
    grid_x = mouse_x // const.TILE_SIZE
    grid_y = const.MAP_HEIGHT - 1 - mouse_y // const.TILE_SIZE

    # Check if the click is on a corner cell
    for corner, (cx, cy) in const.CORNER_POSITIONS.items():
        if grid_x == cx and grid_y == cy and active_corner is None:
            active_corner = corner
            pose_x_offset, pose_y_offset = cx, cy
            active_orientation = None  # Reset active orientation
            print(f"Selected corner: {corner}")
            return

    # Check if the click is on an orientation cell
    if active_corner:
        for i, (ox, oy) in enumerate(const.ORIENTATION_OFFSETS[active_corner]):
            orientation_x = const.CORNER_POSITIONS[active_corner][0] + ox
            orientation_y = const.CORNER_POSITIONS[active_corner][1] + oy
            if grid_x == orientation_x and grid_y == orientation_y:
                active_orientation = (active_corner, i)
                orientation_offset = const.ORIENTATIONS[active_corner][i]
                print(f"Selected orientation: {robot_theta} radians")
                selection_complete = True  # Mark selection as complete
                return


# Function to draw the tables for surface and buried mines
def draw_tables(
    surface_table: pygame.Surface,
    buried_table_surface: pygame.Surface,
) -> None:
    surface_table.fill((255, 255, 255))  # White background
    buried_table_surface.fill((255, 255, 255))  # White background
    pygame.draw.rect(
        surface_table, (0, 0, 0), (0, 0, const.TABLE_WIDTH, const.TABLE_HEIGHT), 2
    )  # Border for surface table
    pygame.draw.rect(
        buried_table_surface,
        (0, 0, 0),
        (0, 0, const.TABLE_WIDTH, const.TABLE_HEIGHT),
        2,
    )  # Border for buried table

    y_offset = const.TABLE_PADDING
    for y in range(const.MAP_HEIGHT):
        for x in range(const.MAP_WIDTH):
            if map_data[y][x] == MINE_TYPE.SURFACE:
                mine_type = "Surface"
                text = FONT.render(
                    f"{chr(65 + x)},{y + 1}: {mine_type}", True, (0, 0, 0)
                )  # Convert index to char
                surface_table.blit(text, (const.TABLE_PADDING, y_offset))
                y_offset += const.FONT_SIZE + 2
                if y_offset > const.TABLE_HEIGHT - const.FONT_SIZE:
                    return
    y_offset = const.TABLE_PADDING
    for y in range(const.MAP_HEIGHT):
        for x in range(const.MAP_WIDTH):
            if map_data[y][x] == MINE_TYPE.BURIED:
                mine_type = "Buried"
                text = FONT.render(
                    f"{chr(65 + x)},{y + 1}: {mine_type}", True, (0, 0, 0)
                )  # Convert index to char
                buried_table_surface.blit(text, (const.TABLE_PADDING, y_offset))
                y_offset += const.FONT_SIZE + 2
                if y_offset > const.TABLE_HEIGHT - const.FONT_SIZE:
                    return


# Function to save the map as a screenshot
def save_map_screenshot(
    directory: str,
    screen: pygame.Surface,
) -> None:
    try:
        os.makedirs(directory, exist_ok=True)

        # Generate a unique filename with a timestamp
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"minefield_map_{timestamp}.png"
        filepath = os.path.join(directory, filename)

        # Save the screenshot
        pygame.image.save(screen, filepath)
        rospy.loginfo(f"Map screenshot saved as '{filepath}'")
    except Exception as e:
        rospy.logerr(f"Failed to save map screenshot: {e}")


def main() -> int:
    # Create the display
    screen = pygame.display.set_mode(
        (
            const.MAP_WIDTH * const.TILE_SIZE
            + 2 * (const.TABLE_WIDTH + const.TABLE_SPACING),
            const.MAP_HEIGHT * const.TILE_SIZE,
        )
    )
    # Create the table surfaces
    surface_table_surface = pygame.Surface((const.TABLE_WIDTH, const.TABLE_HEIGHT))
    buried_table_surface = pygame.Surface((const.TABLE_WIDTH, const.TABLE_HEIGHT))

    # Initialize ROS node
    rospy.init_node("minefield_map", anonymous=False)
    rospy.Subscriber("pose_combined", Pose2D, robot_pose_callback)
    rospy.Subscriber("camera_detection", Bool, camera_detection_callback)
    rospy.Subscriber("detection", Bool, metal_detector_callback)

    rate = rospy.Rate(10)  # 10 Hz

    running = True
    while running and not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # Handle click on map (corner or orientation)
                handle_mouse_click(event.pos)

        mine_detection_callback()

        # Draw the map
        draw_map(screen)

        # Draw the corner and orientation cells
        draw_corner_orientation_cells(screen)

        # Draw the tables
        draw_tables(surface_table_surface, buried_table_surface)
        screen.blit(
            surface_table_surface,
            (const.MAP_WIDTH * const.TILE_SIZE + const.TABLE_SPACING, 0),
        )
        screen.blit(
            buried_table_surface,
            (
                const.MAP_WIDTH * const.TILE_SIZE
                + const.TABLE_SPACING
                + const.TABLE_WIDTH
                + const.TABLE_SPACING,
                0,
            ),
        )

        pygame.display.flip()
        rate.sleep()
    save_map_screenshot("/home/ahmed/Map_Screenshots", screen)
    pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
