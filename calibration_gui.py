
import pygame
import requests
import sys
import time

class CalibrationGUI:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((800, 600))
        pygame.display.set_caption("IMU Calibration")
        self.font = pygame.font.Font(None, 36)
        self.clock = pygame.time.Clock()
        
        self.steps = [
            ("Time Calibration", "Keep IMU stationary", self.calibrate_time),
            ("Gravity Calibration", "Place IMU flat on surface", self.calibrate_gravity),
            ("Gyroscope Calibration", "Keep IMU stationary", self.calibrate_gyro),
            ("Magnetometer Calibration", "Rotate IMU in all directions", self.calibrate_mag)
        ]
        self.current_step = 0
        
    def draw_text(self, text, pos, color=(255, 255, 255)):
        surface = self.font.render(text, True, color)
        rect = surface.get_rect(center=pos)
        self.screen.blit(surface, rect)
        
    def calibrate_time(self):
        response = requests.get('http://localhost:5000/calibrate/time')
        return response.json()['offset_us']
        
    def calibrate_gravity(self):
        return requests.get('http://localhost:5000/calibrate/gravity').json()
        
    def calibrate_gyro(self):
        return requests.get('http://localhost:5000/calibrate/gyro').json()
        
    def calibrate_mag(self):
        return requests.get('http://localhost:5000/calibrate/mag').json()
        
    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                    if self.current_step < len(self.steps):
                        title, instructions, func = self.steps[self.current_step]
                        try:
                            result = func()
                            self.current_step += 1
                        except Exception as e:
                            print(f"Calibration error: {e}")
                    
            self.screen.fill((0, 0, 0))
            
            if self.current_step < len(self.steps):
                title, instructions, _ = self.steps[self.current_step]
                self.draw_text(title, (400, 200))
                self.draw_text(instructions, (400, 300))
                self.draw_text("Press SPACE to continue", (400, 500))
            else:
                self.draw_text("Calibration Complete!", (400, 300))
                
            pygame.display.flip()
            self.clock.tick(60)
            
        pygame.quit()

if __name__ == '__main__':
    gui = CalibrationGUI()
    gui.run()
