import curses
import time
import pyfiglet
from typing import List
from dataclasses import dataclass

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

class CombatSystem:
    def __init__(self):
        self.shield_enabled = False
        self.wounds = 0
        self.max_wounds = 3  # Robot becomes disabled at 3 wounds
        self.laser_available = True
        self.laser_cooldown = 5.0  # 5 second cooldown
        self.last_laser_use = 0

    def toggle_shield(self):
        self.shield_enabled = not self.shield_enabled

    def add_wound(self):
        if self.wounds < self.max_wounds:
            self.wounds += 1

    def repair_wound(self):
        if self.wounds > 0:
            self.wounds -= 1

    @property
    def is_disabled(self):
        return self.wounds >= self.max_wounds

    def use_laser(self):
        if self.laser_available and not self.is_disabled:
            self.laser_available = False
            self.last_laser_use = time.time()

    def update_laser_status(self):
        if not self.laser_available:
            elapsed = time.time() - self.last_laser_use
            if elapsed >= self.laser_cooldown:
                self.laser_available = True
            return max(0, self.laser_cooldown - elapsed)
        return 0

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
        if not self.combat.is_disabled:
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
        self.setup_screen()

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
        if self.robot.combat.is_disabled:
            status_text = "SYSTEM STATUS: DISABLED"
            status_color = ColorScheme.CRITICAL
        else:
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

        # Wounds Status
        wounds_text = f"WOUNDS: {self.robot.combat.wounds}/{self.robot.combat.max_wounds}"
        if self.robot.combat.is_disabled:
            wounds_color = ColorScheme.CRITICAL
            self.stdscr.addstr(current_y, 5, wounds_text + " - CRITICAL DAMAGE", 
                             curses.color_pair(wounds_color) | curses.A_BOLD)
        else:
            wounds_color = (ColorScheme.WARNING if self.robot.combat.wounds > 0 
                          else ColorScheme.STATUS_ON)
            self.stdscr.addstr(current_y, 5, wounds_text, 
                             curses.color_pair(wounds_color) | curses.A_BOLD)
        current_y += 2

        # Laser Status
        if self.robot.combat.is_disabled:
            laser_text = "LASER SYSTEMS OFFLINE"
            laser_color = ColorScheme.CRITICAL
        else:
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
        
        if self.robot.status and not self.robot.combat.is_disabled:
            mode_ascii = self.get_scaled_font(self.robot.current_mode, max_width)
            mode_color = (ColorScheme.OFFENSE_MODE 
                         if self.robot.current_mode == "Offense" 
                         else ColorScheme.DISRUPTOR_MODE)
            
            for line in mode_ascii.splitlines():
                if len(line.strip()) > 0:
                    self.stdscr.addstr(current_y, 5, line, 
                                     curses.color_pair(mode_color) | curses.A_BOLD)
                current_y += 1

            current_y += 1
            mode_info = ("OFFENSIVE OPERATIONS ACTIVE" if self.robot.current_mode == "Offense" 
                        else "DISRUPTION PROTOCOL ENGAGED")
            self.stdscr.addstr(current_y, 5, mode_info, 
                             curses.color_pair(mode_color) | curses.A_BOLD)
            current_y += 2

            direction = self.robot.directions[self.robot.direction_index]
        else:
            if self.robot.combat.is_disabled:
                direction = "MOBILITY SYSTEMS OFFLINE"
            else:
                direction = "Still"

        self.stdscr.addstr(current_y, 5, f"Direction: {direction}", 
                          curses.color_pair(ColorScheme.DEFAULT_MODE) | curses.A_BOLD)
        return current_y + 2

    def handle_key_event(self, key: int) -> bool:
        if key == ord('q'):
            return True
        elif key == ord(' '):
            if not self.robot.combat.is_disabled:
                self.robot.toggle()
        elif key == ord('m') and self.robot.status and not self.robot.combat.is_disabled:
            self.robot.switch_mode()
        elif key == ord('s') and self.robot.status and not self.robot.combat.is_disabled:
            self.robot.combat.toggle_shield()
        elif key == ord('l') and self.robot.status and self.robot.combat.laser_available and not self.robot.combat.is_disabled:
            self.robot.combat.use_laser()
        elif key == ord('w'):  # Add wound
            self.robot.combat.add_wound()
        elif key == ord('r'):  # Repair wound
            self.robot.combat.repair_wound()
        return False

    def main_loop(self):
        while True:
            self.stdscr.clear()
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
                    "[S] Toggle Shield",
                    "[L] Fire Laser",
                    "[W] Add Wound",
                    "[R] Repair Wound",
                    "[Q] Quit"
                ]
                controls_text = " | ".join(controls)
                self.stdscr.addstr(height-2, 5, controls_text, 
                                 curses.color_pair(ColorScheme.DEFAULT_MODE) | curses.A_BOLD)
            except curses.error:
                pass
            
            if self.robot.status and not self.robot.combat.is_disabled:
                self.robot.update_cycles()
            
            self.stdscr.refresh()
            
            key = self.stdscr.getch()
            if self.handle_key_event(key):
                break
            
            time.sleep(0.05)

def main(stdscr):
    interface = Interface(stdscr)
    interface.main_loop()

if __name__ == "__main__":
    curses.wrapper(main)
