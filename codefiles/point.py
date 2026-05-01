
import math
with open('/home/dx/ros2_ws/A/CPE631-Navigation-ROS2-main/maps/cafe.pgm','rb') as f:
    f.readline()
    while True:
        line = f.readline().strip()
        if not line.startswith(b'#'): break
    w, h = map(int, line.split())
    f.readline()
    data = f.read()
ox, oy, res = -5.999, -11.076, 0.05
def val(wx, wy):
    cc = int((wx-ox)/res); cr = h-1-int((wy-oy)/res)
    return data[cr*w+cc] if 0<=cr<h and 0<=cc<w else 0

route = [(0.5,-8.0),(-4.0,0.0),(2.5,8.0),(-3.5,5.5),(2.5,-5.0)]
print('=== Zigzag route verification ===')
for i,(x,y) in enumerate(route):
    nx,ny = route[(i+1)%len(route)]
    yaw = math.atan2(ny-y,nx-x)
    d = math.hypot(nx-x,ny-y)
    v = val(x,y)
    print(f'  G{i+1}: ({x:5.1f},{y:5.1f}) val={v} yaw={yaw:5.2f} -> next {d:.1f}m')
s = ';'.join(f'{x},{y},{math.atan2(route[(i+1)%len(route)][1]-y,route[(i+1)%len(route)][0]-x):.2f}' for i,(x,y) in enumerate(route))
print(f'\nROUTE=\"{s}\"')
"