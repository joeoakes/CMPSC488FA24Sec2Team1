import curses
import time
import pyfiglet
import os
import subprocess
from typing import List
from dataclasses import dataclass
from collections import deque
from datetime import datetime

# Create or clear the input/output files
with open("inputs.txt", "w") as f:
    f.write("")  # Create empty file
with open("outputs.txt", "w") as f:
    f.write("")  # Create empty file

@dataclass
class ColorScheme:
    BACKGROUND = 1
    STATUS_ON = 2
    STATUS_OFF = 3
    DEFAULT_MODE = 4
    OFFENSE_MODE = 5
    DISRUPTOR_MODE = 6
    WARNING = 7
    CRITICAL = 8

def follow(thefile):
    # Start at beginning of file
    thefile.seek(0)
    while True:
        line = thefile.readline()
        if not line:
            # No new line, sleep briefly and try again
            yield None
        # Return the line stripped of newlines
        if line.strip():  # Only yield non-empty lines
            yield line.strip()

class CombatSystem:
    def __init__(self):
        self.shield_enabled = False
        self.laser_available = True
        self.laser_cooldown = 5.0  # 5 second cooldown
        self.last_laser_use = 0
        self.hit_log = deque(maxlen=5)  # Keep last 5 hits
        self.output_file = open("outputs.txt", "a")
        self.script_running = False

    def __del__(self):
        if hasattr(self, 'output_file'):
            self.output_file.close()

    def set_shield(self, enabled: bool):
        self.shield_enabled = enabled

    def process_hit(self):
        timestamp = datetime.now().strftime("%H:%M:%S")
        if not self.shield_enabled:
            self.hit_log.append(f"{timestamp} - HIT TAKEN")
            self.output_file.write("hit_taken\n")
            self.output_file.flush()
        else:
            self.hit_log.append(f"{timestamp} - HIT BLOCKED BY SHIELD")

    def try_fire_laser(self) -> bool:
        if self.laser_available:
            self.laser_available = False
            self.last_laser_use = time.time()
            self.output_file.write("laser_fired\n")
            self.output_file.flush()
            return True
        return False

    def update_laser_status(self):
        if not self.laser_available:
            elapsed = time.time() - self.last_laser_use
            if elapsed >= self.laser_cooldown:
                self.laser_available = True
            return max(0, self.laser_cooldown - elapsed)
        return 0

    def start_script(self):
        if not self.script_running:
            self.script_running = True
            try:
                subprocess.Popen(["./start.sh"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            except Exception as e:
                self.hit_log.append(f"{datetime.now().strftime('%H:%M:%S')} - Script Error: {str(e)}")
            finally:
                self.script_running = False

class RobotStatus:
    def __init__(self):
        self.status = False
        self.current_mode = "Offense"
        self.directions = ["Forward", "Back", "Turning Left", "Turning Right", "Diagonal Left", "Diagonal Right"]
        self.direction_index = 0
        self.cycle_interval = 1.0
        self.last_direction_cycle = time.time()
        self.combat = CombatSystem()

    def toggle(self):
        self.status = not self.status

    def switch_mode(self):
        self.current_mode = "Disruptor" if self.current_mode == "Offense" else "Offense"

    def update_cycles(self):
        current_time = time.time()
        if current_time - self.last_direction_cycle >= self.cycle_interval:
            self.direction_index = (self.direction_index + 1) % len(self.directions)
            self.last_direction_cycle = current_time

class Interface:
    def __init__(self, stdscr):
        self.stdscr = stdscr
        self.robot = RobotStatus()
        self.command_file = open("inputs.txt", "r")
        self.command_lines = follow(self.command_file)
        self.setup_screen()

    def __del__(self):
        if hasattr(self, 'command_file'):
            self.command_file.close()

    def setup_screen(self):
        curses.curs_set(0)
        self.stdscr.nodelay(1)
        curses.start_color()
        
        curses.init_pair(ColorScheme.BACKGROUND, curses.COLOR_WHITE, curses.COLOR_BLACK)
        curses.init_pair(ColorScheme.STATUS_ON, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(ColorScheme.STATUS_OFF, curses.COLOR_RED, curses.COLOR_BLACK)
        curses.init_pair(ColorScheme.DEFAULT_MODE, curses.COLOR_CYAN, curses.COLOR_BLACK)
        curses.init_pair(ColorScheme.OFFENSE_MODE, curses.COLOR_RED, curses.COLOR_BLACK)
        curses.init_pair(ColorScheme.DISRUPTOR_MODE, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(ColorScheme.WARNING, curses.COLOR_YELLOW, curses.COLOR_BLACK)
        curses.init_pair(ColorScheme.CRITICAL, curses.COLOR_MAGENTA, curses.COLOR_BLACK)

    def get_scaled_font(self, text: str, max_width: int) -> str:
        fonts = ['big', 'standard', 'small']
        for font in fonts:
            try:
                ascii_art = pyfiglet.figlet_format(text, font=font)
                if max(len(line) for line in ascii_art.splitlines()) <= max_width - 10:
                    return ascii_art
            except Exception:
                continue
        return pyfiglet.figlet_format(text, font='small')

    def draw_background(self):
        height, width = self.stdscr.getmaxyx()
        for y in range(height):
            try:
                self.stdscr.addstr(y, 0, " " * width, curses.color_pair(ColorScheme.BACKGROUND))
            except curses.error:
                pass

    def draw_status_indicator(self, start_y: int) -> int:
        status_text = "SYSTEM STATUS: ON" if self.robot.status else "SYSTEM STATUS: OFF"
        status_color = ColorScheme.STATUS_ON if self.robot.status else ColorScheme.STATUS_OFF
        self.stdscr.addstr(start_y, 5, status_text, curses.color_pair(status_color) | curses.A_BOLD)
        return start_y + 2

    def draw_combat_status(self, start_y: int) -> int:
        current_y = start_y

        # Shield Status
        shield_status = "ENABLED" if self.robot.combat.shield_enabled else "DISABLED"
        shield_color = ColorScheme.STATUS_ON if self.robot.combat.shield_enabled else ColorScheme.STATUS_OFF
        self.stdscr.addstr(current_y, 5, f"SHIELD STATUS: {shield_status}", 
                          curses.color_pair(shield_color) | curses.A_BOLD)
        current_y += 2

        # Hit Log
        self.stdscr.addstr(current_y, 5, "HIT LOG:", curses.color_pair(ColorScheme.DEFAULT_MODE) | curses.A_BOLD)
        current_y += 1
        if not self.robot.combat.hit_log:
            self.stdscr.addstr(current_y, 5, "No hits recorded", curses.color_pair(ColorScheme.DEFAULT_MODE))
            current_y += 1
        else:
            for hit in self.robot.combat.hit_log:
                color = ColorScheme.WARNING if "HIT TAKEN" in hit else ColorScheme.STATUS_ON
                self.stdscr.addstr(current_y, 5, hit, curses.color_pair(color) | curses.A_BOLD)
                current_y += 1
        current_y += 1

        # Laser Status
        cooldown = self.robot.combat.update_laser_status()
        if self.robot.combat.laser_available:
            laser_text = "LASER READY"
            laser_color = ColorScheme.STATUS_ON
        else:
            laser_text = f"LASER COOLDOWN: {cooldown:.1f}s"
            laser_color = ColorScheme.WARNING
        
        self.stdscr.addstr(current_y, 5, laser_text, 
                          curses.color_pair(laser_color) | curses.A_BOLD)
        current_y += 2

        return current_y

    def draw_mode_and_status(self, start_y: int, max_width: int) -> int:
        current_y = start_y
        
        if self.robot.status:
            mode_ascii = self.get_scaled_font(self.robot.current_mode, max_width)
            mode_color = ColorScheme.OFFENSE_MODE if self.robot.current_mode == "Offense" else ColorScheme.DISRUPTOR_MODE
            
            for line in mode_ascii.splitlines():
                if len(line.strip()) > 0:
                    self.stdscr.addstr(current_y, 5, line, 
                                     curses.color_pair(mode_color) | curses.A_BOLD)
                current_y += 1

            current_y += 1
            mode_info = "OFFENSIVE OPERATIONS ACTIVE" if self.robot.current_mode == "Offense" else "DISRUPTION PROTOCOL ENGAGED"
            self.stdscr.addstr(current_y, 5, mode_info, 
                             curses.color_pair(mode_color) | curses.A_BOLD)
            current_y += 2

            direction = self.robot.directions[self.robot.direction_index]
        else:
            direction = "Still"

        self.stdscr.addstr(current_y, 5, f"Direction: {direction}", 
                          curses.color_pair(ColorScheme.DEFAULT_MODE) | curses.A_BOLD)
        return current_y + 2

    def check_commands(self):
        try:
            line = next(self.command_lines)
            if line:
                line = line.lower()
                if line == "hit":
                    self.robot.combat.process_hit()
                elif line == "shield_on":
                    self.robot.combat.set_shield(True)
                elif line == "shield_off":
                    self.robot.combat.set_shield(False)
                elif line == "fire_laser":
                    self.robot.combat.try_fire_laser()
        except (StopIteration, OSError):
            pass

    def handle_key_event(self, key: int) -> bool:
        if key == ord('q'):
            return True
        elif key == ord(' '):
            self.robot.toggle()
        elif key == ord('m') and self.robot.status:
            self.robot.switch_mode()
        elif key == ord('s') and self.robot.status and not self.robot.combat.script_running:
            self.robot.combat.start_script()
        return False

    def main_loop(self):
        try:
            while True:
                try:
                    self.stdscr.erase()
                    height, width = self.stdscr.getmaxyx()
                    
                    self.draw_background()
                    
                    # Draw title
                    title_ascii = self.get_scaled_font("ROBOT STATUS", width)
                    current_y = 1
                    for line in title_ascii.splitlines():
                        if len(line.strip()) > 0:
                            try:
                                self.stdscr.addstr(current_y, 5, line, 
                                                 curses.color_pair(ColorScheme.DEFAULT_MODE) | curses.A_BOLD)
                            except curses.error:
                                pass
                        current_y += 1
                    
                    current_y += 1
                    current_y = self.draw_status_indicator(current_y)
                    current_y = self.draw_combat_status(current_y)
                    current_y = self.draw_mode_and_status(current_y, width)
                    
                    # Draw controls
                    try:
                        controls = [
                            "[SPACE] Toggle On/Off",
                            "[M] Switch Mode",
                            "[S] Start Script" if self.robot.status else "",
                            "[Q] Quit"
                        ]
                        controls = [c for c in controls if c]  # Remove empty controls
                        controls_text = " | ".join(controls)
                        self.stdscr.addstr(height-2, 5, controls_text, 
                                         curses.color_pair(ColorScheme.DEFAULT_MODE) | curses.A_BOLD)
                    except curses.error:
                        pass
                    
                    if self.robot.status:
                        self.robot.update_cycles()
                    
                    # Check for incoming commands
                    self.check_commands()
                    
                    self.stdscr.refresh()

                    key = self.stdscr.getch()
                    if self.handle_key_event(key):
                        break
                    
                    time.sleep(0.01)
                
                except curses.error:
                    # Handle terminal resize or other curses errors
                    self.stdscr.refresh()
                    continue
                    
        finally:
            # Clean up
            if hasattr(self, 'command_file'):
                self.command_file.close()
            if hasattr(self.robot.combat, 'output_file'):
                self.robot.combat.output_file.close()

def main(stdscr):
    interface = Interface(stdscr)
    interface.main_loop()

if __name__ == "__main__":
    curses.wrapper(main)
