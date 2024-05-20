import pygame
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from collections import deque
import random

# 초기 설정
pygame.init()

# 화면 크기 설정
screen_width = 1300
screen_height = 600
screen = pygame.display.set_mode((screen_width, screen_height))

# 색상 설정
white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)
blue = (0, 0, 255)
green = (0, 255, 0)

# 드론 매개변수
drone_x = 250
drone_y = 300
drone_length = 100
drone_angle = 90
drone_angular_velocity = 0

# PID 매개변수 초기값
p_gain = 1.0
i_gain = 0.0
d_gain = 0.05
integral_limit = 5  # I 값 제한

# 외란을 추가할 시간 간격 (밀리초)
disturbance_interval = 8000
last_disturbance_time = 0

# 슬라이더 클래스 정의
class Slider:
    def __init__(self, x, y, w, h, min_val, max_val, start_val, color):
        self.rect = pygame.Rect(x, y, w, h)
        self.min_val = min_val
        self.max_val = max_val
        self.value = start_val
        self.color = color
        self.dragging = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.dragging = True
        elif event.type == pygame.MOUSEBUTTONUP:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION:
            if self.dragging:
                self.value = self.min_val + (self.max_val - self.min_val) * ((event.pos[0] - self.rect.x) / self.rect.w)
                self.value = max(self.min_val, min(self.max_val, self.value))

    def draw(self, screen):
        pygame.draw.rect(screen, self.color, self.rect, 2)
        handle_x = self.rect.x + int((self.value - self.min_val) / (self.max_val - self.min_val) * self.rect.w)
        pygame.draw.circle(screen, self.color, (handle_x, self.rect.y + self.rect.h // 2), self.rect.h // 2)
        # 슬라이더 값 텍스트로 표시
        font = pygame.font.Font(None, 36)
        text = font.render(f'{self.value:.2f}', True, self.color)
        screen.blit(text, (self.rect.x + self.rect.w + 10, self.rect.y))

# 슬라이더 설정
p_slider = Slider(1100, 100, 100, 20, 0.0, 50.0, 1.0, red)
i_slider = Slider(1100, 150, 100, 20, 0.0, 50.0, 0.0, green)
d_slider = Slider(1100, 200, 100, 20, 0.0, 2.0, 0.05, blue)

# PID 제어 함수
def pid_control(target, current, p_gain, i_gain, d_gain, time_delta):
    global previous_error, integral

    error = target - current
    integral += error * time_delta
    integral = max(min(integral, integral_limit), -integral_limit)  # I 값 제한
    derivative = (error - previous_error) / time_delta
    previous_error = error

    return p_gain * error + i_gain * integral + d_gain * derivative

# 초기화
previous_error = 0
integral = 0
simulation_clock = pygame.time.Clock()
pid_clock = pygame.time.Clock()
display_clock = pygame.time.Clock()

# 자세 데이터 저장
angle_history = deque()
time_history = deque()
start_time = pygame.time.get_ticks()
current_time = start_time

# 메인 루프
running = True
while running:
    # 시뮬레이션 루프 (240Hz)
    time_delta = simulation_clock.tick(100) / 1000.0
    current_time = pygame.time.get_ticks()
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        p_slider.handle_event(event)
        i_slider.handle_event(event)
        d_slider.handle_event(event)
        
    # 슬라이더 값 업데이트
    p_gain = p_slider.value
    i_gain = i_slider.value
    d_gain = d_slider.value
    
    if current_time - last_disturbance_time > disturbance_interval:
        drone_angular_velocity += 200
        last_disturbance_time = current_time
    drone_angular_velocity+=10 #constant disturb
    
    if pid_clock.tick(50):
        # 목표 각도와 현재 각도 설정 (여기서는 예제로 목표 각도를 0으로 설정)
        target_angle = 0
        # 랜덤 노이즈 추가
        noise = random.uniform(-0.1, 0.1)  # 랜덤 측정 노이즈
        control_signal = pid_control(target_angle, drone_angle + noise, p_gain, i_gain, d_gain, time_delta)
    
        drone_angular_velocity += control_signal
    
    drone_angle += drone_angular_velocity * time_delta
    
    # 자세 데이터 저장
    elapsed_time = (current_time - start_time) / 1000.0
    angle_history.append(drone_angle)
    time_history.append(elapsed_time)
    
    # 최근 5초 데이터만 유지
    while time_history and (elapsed_time - time_history[0]) > 5:
        time_history.popleft()
        angle_history.popleft()
    
    # 화면 업데이트 (30Hz)
    if display_clock.tick(30):
        # 화면 지우기
        screen.fill(white)
        
        # 드론 그리기 (양쪽에 막대기와 원 추가)
        end_x1 = drone_x + drone_length * np.cos(np.radians(drone_angle))
        end_y1 = drone_y + drone_length * np.sin(np.radians(drone_angle))
        end_x2 = drone_x - drone_length * np.cos(np.radians(drone_angle))
        end_y2 = drone_y - drone_length * np.sin(np.radians(drone_angle))
        pygame.draw.line(screen, black, (drone_x, drone_y), (end_x1, end_y1), 5)
        pygame.draw.line(screen, black, (drone_x, drone_y), (end_x2, end_y2), 5)
        pygame.draw.circle(screen, red, (int(end_x1), int(end_y1)), 10)
        pygame.draw.circle(screen, red, (int(end_x2), int(end_y2)), 10)
        
        # 그래프 그리기
        fig, ax = plt.subplots()
        ax.plot(time_history, angle_history, label='Drone Angle')
        ax.set_xlim([max(0, elapsed_time - 5), elapsed_time])
        y_min = min(angle_history) - 5
        y_max = max(angle_history) + 5
        ax.set_ylim([min(y_min, 0), max(y_max, 0)])
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (degrees)')
        ax.set_title('Drone Angle Over Time')
        ax.legend()
        ax.grid(True)
        
        # Matplotlib figure를 Pygame surface로 변환
        canvas = FigureCanvas(fig)
        canvas.draw()
        renderer = canvas.get_renderer()
        raw_data = renderer.buffer_rgba().tobytes()
        size = canvas.get_width_height()
        graph_surf = pygame.image.fromstring(raw_data, size, "RGBA")
        plt.close(fig)
        
        # 그래프 그리기
        screen.blit(graph_surf, (500, 50))
        
        # 슬라이더 그리기
        p_slider.draw(screen)
        i_slider.draw(screen)
        d_slider.draw(screen)
        
        # 화면 업데이트
        pygame.display.flip()

pygame.quit()
