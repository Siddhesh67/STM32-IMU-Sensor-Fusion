import pygame
import serial
import numpy as np
import sys

SERIAL_PORT = '/dev/tty.usbmodem11403'
BAUD_RATE   = 115200

vertices = np.array([
    [-1, -1, -1], [ 1, -1, -1],
    [ 1,  1, -1], [-1,  1, -1],
    [-1, -1,  1], [ 1, -1,  1],
    [ 1,  1,  1], [-1,  1,  1]
], dtype=float)

edges = [
    (0,1),(1,2),(2,3),(3,0),
    (4,5),(5,6),(6,7),(7,4),
    (0,4),(1,5),(2,6),(3,7)
]

faces = [
    ([0,1,2,3], (255, 80,  80)),
    ([4,5,6,7], (80,  255, 80)),
    ([0,1,5,4], (80,  80,  255)),
    ([2,3,7,6], (255, 255, 80)),
    ([0,3,7,4], (255, 160, 80)),
    ([1,2,6,5], (160, 80,  255)),
]

def rotation_matrix(roll_deg, pitch_deg):
    r = np.radians(roll_deg)
    p = np.radians(pitch_deg)
    Rx = np.array([
        [1,      0,       0],
        [0, np.cos(p), -np.sin(p)],
        [0, np.sin(p),  np.cos(p)]
    ])
    Rz = np.array([
        [np.cos(r), -np.sin(r), 0],
        [np.sin(r),  np.cos(r), 0],
        [0,          0,         1]
    ])
    return Rz @ Rx

def project(vertex, width, height, scale=120):
    x = vertex[0] * scale + width  // 2
    y = vertex[1] * scale + height // 2
    return (int(x), int(y))

def draw_cube(screen, roll, pitch, width, height):
    R = rotation_matrix(roll, pitch)
    rotated = [R @ v for v in vertices]
    projected = [project(v, width, height) for v in rotated]
    face_depths = []
    for face_verts, color in faces:
        depth = sum(rotated[i][2] for i in face_verts) / 4
        face_depths.append((depth, face_verts, color))
    face_depths.sort(key=lambda x: x[0])
    for depth, face_verts, color in face_depths:
        points = [projected[i] for i in face_verts]
        pygame.draw.polygon(screen, color, points)
        pygame.draw.polygon(screen, (0,0,0), points, 2)

def main():
    pygame.init()
    WIDTH, HEIGHT = 800, 600
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("MPU6050 Sensor Fusion Visualizer")
    font  = pygame.font.SysFont('Arial', 24)
    clock = pygame.time.Clock()

    roll  = 0.0
    pitch = 0.0

    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to {SERIAL_PORT}")
    except:
        print("Serial port not found - running in demo mode")

    running = True
    demo_angle = 0.0

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

        if ser and ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8').strip()
                if 'Roll:' in line and 'Pitch:' in line:
                    parts = line.split()
                    roll  = float(parts[1])
                    pitch = float(parts[3])
            except:
                pass
        elif not ser:
            demo_angle += 0.5
            roll  = 30 * np.sin(np.radians(demo_angle))
            pitch = 20 * np.cos(np.radians(demo_angle * 0.7))

        screen.fill((30, 30, 30))
        draw_cube(screen, roll, pitch, WIDTH, HEIGHT)

        roll_text  = font.render(f"Roll:  {roll:6.1f}", True, (255,255,255))
        pitch_text = font.render(f"Pitch: {pitch:6.1f}", True, (255,255,255))
        mode_text  = font.render(
            "LIVE" if ser else "DEMO MODE",
            True,
            (80,255,80) if ser else (255,200,80)
        )
        screen.blit(roll_text,  (20, 20))
        screen.blit(pitch_text, (20, 50))
        screen.blit(mode_text,  (20, 80))

        pygame.display.flip()
        clock.tick(60)

    if ser:
        ser.close()
    pygame.quit()
    sys.exit()

if __name__ == '__main__':
    main()
