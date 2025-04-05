
import pygame
import sys
from imu_api import IMUAPI

class CalibrationGUI:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((800, 600))
        pygame.display.set_caption("IMU Calibration")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 36)
        self.imu = IMUAPI()
        self.status = "Ready"
        self.buttons = [
            {"text": "Time Calibration", "action": self.calibrate_time},
            {"text": "Still Calibration (Gravity+Gyro)", "action": self.calibrate_still},
            {"text": "Mag Calibration", "action": self.calibrate_mag}
        ]

    def draw_button(self, text: str, pos: tuple, size: tuple) -> pygame.Rect:
        rect = pygame.Rect(pos, size)
        pygame.draw.rect(self.screen, (100, 100, 100), rect)
        text_surf = self.font.render(text, True, (255, 255, 255))
        text_rect = text_surf.get_rect(center=rect.center)
        self.screen.blit(text_surf, text_rect)
        return rect

    def draw_status(self):
        text = self.font.render(self.status, True, (255, 255, 255))
        self.screen.blit(text, (20, 550))

    def calibrate_time(self):
        try:
            offset = self.imu.calibrate_time()
            self.status = f"Time offset: {offset} μs"
        except Exception as e:
            self.status = f"Error: {str(e)}"

    def calibrate_still(self):
        try:
            self.status = f"Keep IMU still for {self.imu.still_delay} seconds..."
            pygame.display.flip()
            time.sleep(self.imu.still_delay)
            self.imu.calibrate_still_sensors()
            self.status = "Still calibration complete"
        except Exception as e:
            self.status = f"Error: {str(e)}"

    def calibrate_mag(self):
        try:
            self.status = f"Get ready to rotate IMU for {self.imu.rotation_delay} seconds..."
            pygame.display.flip()
            time.sleep(self.imu.rotation_delay)
            self.imu.calibrate_mag()
            self.status = "Magnetometer calibration complete"
        except Exception as e:
            self.status = f"Error: {str(e)}"

    def run(self):
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                if event.type == pygame.MOUSEBUTTONDOWN:
                    pos = pygame.mouse.get_pos()
                    for i, button in enumerate(self.buttons):
                        rect = pygame.Rect(50, 100 + i*80, 700, 60)
                        if rect.collidepoint(pos):
                            button["action"]()

            self.screen.fill((50, 50, 50))
            for i, button in enumerate(self.buttons):
                self.draw_button(button["text"], (50, 100 + i*80), (700, 60))
            self.draw_status()
            pygame.display.flip()
            self.clock.tick(60)

if __name__ == "__main__":
    gui = CalibrationGUI()
    gui.run()
