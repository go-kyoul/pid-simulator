import pygame
import numpy as np
from collections import deque
import random

# 초기 설정
pygame.init()

# 화면 크기 설정
screen_width = 1300
screen_height = 450
screen = pygame.display.set_mode((screen_width, screen_height))

# 색상 설정
white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)
blue = (0, 0, 255)
green = (0, 255, 0)
gray = (150, 150, 150)
heavy_gray = (100, 100, 100)

# 드론 매개변수
drone_x = 200
drone_y = 200
drone_length = 100
drone_angle = 90
drone_angular_velocity = 0

# PID 매개변수 초기값
p_gain = 1.0
i_gain = 0.0
d_gain = 0.05
integral_limit = 8  # I 값 제한

# 외란을 추가할 시간 간격 (밀리초)
disturbance_interval = 8000
last_disturbance_time = 0

# setpoint를 변경할 시간 간격 (밀리초)
setpoint_interval = 2000
last_setpoint_time = 0
setpoint_values = [-30, 0, 30]
setpoint_index = 1

# 슬라이더 클래스 정의
class Slider:
    def __init__(self, x, y, w, h, min_val, max_val, start_val, color, label):
        self.rect = pygame.Rect(x, y, w, h)
        self.min_val = min_val
        self.max_val = max_val
        self.value = start_val
        self.color = color
        self.dragging = False
        self.label = label

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
        text = font.render(self.label, True, self.color)
        screen.blit(text, (self.rect.x - 35, self.rect.y))

# 슬라이더 설정
p_slider = Slider(1050, 100, 100, 20, 0.0, 60.0, 1.0, red, 'P')
i_slider = Slider(1050, 150, 100, 20, 0.0, 50.0, 0.0, green, 'I')
d_slider = Slider(1050, 200, 100, 20, 0.0, 5.0, 0.05, blue, 'D')

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

# setpoint 데이터 저장
setpoint_history = deque()

# 각 루프 주기 (밀리초)
simulation_interval = 1000.0/100  # 100Hz
pid_interval = 1000.0/30         # 30Hz
display_interval = 1000.0/60  # 60Hz

# 마지막 실행 시간
last_simulation_time = start_time
last_pid_time = start_time
last_display_time = start_time

# 메인 루프
running = True
while running:
    current_time = pygame.time.get_ticks()
    
    # 시뮬레이션 루프 (100Hz)
    if current_time - last_simulation_time >= simulation_interval:
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
        time_delta = (current_time - last_simulation_time) / 1000.0
        last_simulation_time = current_time
        
        drone_angular_velocity += 8  # constant disturbance
        
        drone_angle += drone_angular_velocity * time_delta

        # 자세 데이터 저장
        elapsed_time = (current_time - start_time) / 1000.0
        angle_history.append(drone_angle)
        time_history.append(elapsed_time)
        # setpoint 데이터 저장
        setpoint_history.append(setpoint_values[setpoint_index])
        
        # 최근 5초 데이터만 유지
        while time_history and (elapsed_time - time_history[0]) > 5:
            time_history.popleft()
            angle_history.popleft()
            setpoint_history.popleft()
    
    # PID 제어 루프 (30Hz)
    if current_time - last_pid_time >= pid_interval:
        pid_time_delta = (current_time - last_pid_time) / 1000.0
        last_pid_time = current_time
        
        target_angle = setpoint_values[setpoint_index]
        noise = random.uniform(-0.1, 0.1)  # 랜덤 측정 노이즈
        control_signal = 1.5 * pid_control(target_angle, drone_angle + noise, p_gain, i_gain, d_gain, pid_time_delta)
        control_signal = max(min(control_signal, 120), -120)  # control_signal 값을 -120에서 120 사이로 제한
        drone_angular_velocity += control_signal

    # setpoint 변경
    if current_time - last_setpoint_time >= setpoint_interval:
        last_setpoint_time = current_time
        setpoint_index = (setpoint_index + 1) % len(setpoint_values)

    
    # 화면 업데이트 (60Hz)
    if current_time - last_display_time >= display_interval:
        last_display_time = current_time
        
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
        graph_width = 500
        graph_height = 300
        graph_x = 450
        graph_y = 50

        # 그래프 배경
        pygame.draw.rect(screen, black, (graph_x, graph_y, graph_width, graph_height), 3)
        
        if len(time_history) > 1:
            max_time = max(time_history)
            min_time = max(0, max_time - 5)
            mean = ((min(min(angle_history), 0))+max(max(angle_history), 0))/2.0
            min_angle = min(min(angle_history), 0) - (mean - min(min(angle_history), 0)) * 0.5   # y축에 0을 포함
            max_angle = max(max(angle_history), 0) + (max(max(angle_history), 0) - mean) * 0.5   # y축에 0을 포함
            angle_range = max_angle - min_angle if max_angle != min_angle else 1


            points = []
            for t, angle in zip(time_history, angle_history):
                if t >= min_time:
                    x = graph_x + graph_width * (t - min_time) / 5
                    y = graph_y + graph_height - (graph_height * (angle - min_angle) / angle_range)
                    points.append((x, y))
            
            if len(points) > 1:
                pygame.draw.lines(screen, blue, False, points, 2)
            
           
            
            # 0 기준선 그리기
            zero_y = graph_y + graph_height - (graph_height * (0 - min_angle) / angle_range)
           
            pygame.draw.line(screen, heavy_gray, (graph_x, zero_y), (graph_x + graph_width, zero_y), 2)

             # setpoint 그래프 그리기
            setpoint_points = []
            for t, sp in zip(time_history, setpoint_history):
                if t >= min_time:
                    x = graph_x + graph_width * (t - min_time) / 5
                    y = graph_y + graph_height - (graph_height * (sp - min_angle) / angle_range)
                    if min_angle <= sp <= max_angle:  # 그래프 범위 내에 있을 때만 그리기
                        setpoint_points.append((x, y))
            
            if len(setpoint_points) > 1:
                pygame.draw.lines(screen, green, False, setpoint_points, 1)

            # 축 및 텍스트
            font = pygame.font.Font(None, 24)

            num_ver = 11
            for i in range(num_ver):
                t = min_time + i * (5 / (num_ver - 1))
                x = graph_x + graph_width * i / (num_ver - 1)
                y = graph_y + graph_height + 5
                text = font.render(f'{t:.1f}', True, black)
                screen.blit(text, (x, y))
                # 세로 눈금선 그리기
                pygame.draw.line(screen, gray, (x, graph_y), (x, graph_y + graph_height), 1)

            num_hor = 7
            for i in range(num_hor):
                angle = min_angle + angle_range * i / (num_hor-1)
                x = graph_x - 40
                y = graph_y + graph_height - (graph_height * i / (num_hor-1))
                text = font.render(f'{angle:.1f}', True, black)
                screen.blit(text, (x, y - 10))
                # 가로 눈금선 그리기
                pygame.draw.line(screen, gray, (graph_x, y), (graph_x + graph_width, y), 1)
        
        # 슬라이더 그리기
        p_slider.draw(screen)
        i_slider.draw(screen)
        d_slider.draw(screen)
        
        # 화면 업데이트
        pygame.display.flip()

pygame.quit()
