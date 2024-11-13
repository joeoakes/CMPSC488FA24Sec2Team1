import curses
import time
import pyfiglet  # Make sure pyfiglet is installed

def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(0)
    curses.start_color()

    # Define color pairs for sections and text
    curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_BLUE)    # Left section background
    curses.init_pair(2, curses.COLOR_WHITE, curses.COLOR_RED)     # Right section background
    curses.init_pair(3, curses.COLOR_GREEN, curses.COLOR_BLACK)   # ON color for text
    curses.init_pair(4, curses.COLOR_RED, curses.COLOR_BLACK)     # OFF color for text
    curses.init_pair(5, curses.COLOR_YELLOW, curses.COLOR_BLUE)   # Default mode color with blue background
    curses.init_pair(6, curses.COLOR_CYAN, curses.COLOR_BLUE)     # Attack mode color with blue background
    curses.init_pair(7, curses.COLOR_MAGENTA, curses.COLOR_BLUE)  # Defense mode color with blue background

    status = False  # Start with the robot OFF
    current_mode = "Attack"  # Default mode is "Attack"
    mode_color = curses.color_pair(6)  # Color for "Attack" mode

    # Object and Direction variables
    objects = ["Wall", "Cone", "Robot"]
    directions = ["Forward", "Back", "Turning Left", "Turning Right"]
    object_index = 0
    direction_index = 0
    cycle_interval = 1  # Interval in seconds for cycling

    # Initialize last update times for cycling
    last_object_cycle_time = time.time()
    last_direction_cycle_time = time.time()

    # Generate large ASCII art text for "ROBOT STATUS" using pyfiglet
    ascii_art_status = pyfiglet.figlet_format("ROBOT STATUS")

    # Enable mouse events
    curses.mousemask(curses.ALL_MOUSE_EVENTS)

    # Fixed x-position for buttons based on your setup
    button_x = 75  # Fixed x-position for buttons
    buttons = [
        {"label": "Toggle ON/OFF", "y": 5, "x": button_x, "action": "toggle"},
        {"label": "Switch Mode", "y": 8, "x": button_x, "action": "switch_mode"},
        {"label": "Quit", "y": 14, "x": button_x, "action": "quit"}
    ]

    def draw_buttons():
        for button in buttons:
            stdscr.addstr(button["y"], button["x"], f"[ {button['label']} ]", curses.color_pair(2))

    while True:
        height, width = stdscr.getmaxyx()
        split_point = min(width // 2, width - 1)  # Ensure split_point is within bounds
        stdscr.clear()

        # Set background color for the left section
        stdscr.bkgd(" ", curses.color_pair(1))  # Set left section's background color
        for y in range(height):
            try:
                stdscr.addstr(y, 0, " " * split_point, curses.color_pair(1))  # Fill left section up to split_point
            except curses.error:
                pass

        # Set background color for the right section
        stdscr.attron(curses.color_pair(2))
        for y in range(height):
            try:
                stdscr.addstr(y, split_point, " " * max(0, width - split_point - 1))  # Ensure within width bounds
            except curses.error:
                pass
        stdscr.attroff(curses.color_pair(2))

        # Draw large ASCII art text for "ROBOT STATUS" line-by-line
        for i, line in enumerate(ascii_art_status.splitlines()):
            stdscr.addstr(1 + i, 5, line, curses.color_pair(5) | curses.A_BOLD)

        # Draw the ON/OFF box below the ASCII art
        stdscr.addstr(8, 5, "###################", curses.color_pair(1))
        stdscr.addstr(9, 5, "#                 #", curses.color_pair(1))
        stdscr.addstr(10, 5, "#                 #", curses.color_pair(1))
        stdscr.addstr(11, 5, "#                 #", curses.color_pair(1))
        stdscr.addstr(12, 5, "#                 #", curses.color_pair(1))
        stdscr.addstr(13, 5, "###################", curses.color_pair(1))

        # Display ON/OFF status in the box
        on_off_text = "ON" if status else "OFF"
        stdscr.addstr(11, 13, on_off_text.center(5), (curses.color_pair(1) if status else curses.color_pair(1)) | curses.A_BOLD)

        # Only display Attack/Defense ASCII art if the robot is ON
        if status:
            ascii_art_mode = pyfiglet.figlet_format(current_mode)
            for i, line in enumerate(ascii_art_mode.splitlines()):
                stdscr.addstr(15 + i, 5, line, mode_color | curses.A_BOLD)

            # Update object and direction when ON
            current_time = time.time()
            if current_time - last_object_cycle_time >= cycle_interval:
                object_index = (object_index + 1) % len(objects)
                last_object_cycle_time = current_time

            if current_time - last_direction_cycle_time >= cycle_interval:
                direction_index = (direction_index + 1) % len(directions)
                last_direction_cycle_time = current_time

            # Display object detection and direction
            stdscr.addstr(20, 5, f"Object Detected: {objects[object_index]}", curses.color_pair(5))
            stdscr.addstr(21, 5, f"Direction Moving: {directions[direction_index]}", curses.color_pair(5))
        else:
            # Display "None" and "Still" when OFF
            stdscr.addstr(20, 5, "Object Detected: None", curses.color_pair(5))
            stdscr.addstr(21, 5, "Direction Moving: Still", curses.color_pair(5))

        # Draw on-screen buttons with the right background color
        draw_buttons()

        stdscr.refresh()

        # Event handling
        key = stdscr.getch()
        
        # Handle mouse click events
        if key == curses.KEY_MOUSE:
            _, mx, my, _, _ = curses.getmouse()
            
            # Check if click is within button boundaries
            for button in buttons:
                if button["y"] == my and button["x"] <= mx <= button["x"] + len(button["label"]) + 3:
                    action = button["action"]
                    if action == "toggle":
                        status = not status
                    elif action == "switch_mode" and status:  # Only switch modes if the robot is ON
                        current_mode = "Defense" if current_mode == "Attack" else "Attack"
                        mode_color = curses.color_pair(6) if current_mode == "Attack" else curses.color_pair(7)
                    elif action == "quit":
                        return  # Exit the program

        elif key == ord('q'):
            break

        time.sleep(0.1)

# Run the GUI
curses.wrapper(main)

