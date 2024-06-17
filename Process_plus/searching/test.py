import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import yaml
import time

# Đọc thông tin từ file YAML
with open('output.yaml', 'r') as file1:
    data = yaml.safe_load(file1)
    agent_name = 'agent3'  # Thay thế bằng tên của agent bạn quan tâm
    agent_schedule = data['schedule'][agent_name]
    steps = len(agent_schedule)

# Tạo môi trường và agent dựa trên thông tin đọc từ YAML
num_agents = len(data['schedule'])
grid_size = 10
grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]

# Màu sắc của các agent
agent_colors = ['red', 'green', 'blue', 'purple', 'orange', 'pink', 'brown', 'gray', 'olive', 'cyan']

agents = {}
for i, (agent_name, agent_data) in enumerate(data['schedule'].items()):
    agents[agent_name] = {'path': agent_data, 'position': {'x': agent_data[0]['x'], 'y': agent_data[0]['y']}, 'color': agent_colors[i]}

# Khởi tạo đồ thị
fig, ax = plt.subplots()
agent_squares = {}

for agent_name, agent_data in agents.items():
    position = agent_data['position']
    square = plt.Rectangle((position['x'], position['y']), 1,1, color='red', label=f'{agent_name[5:]}')
    ax.add_patch(square)
    agent_squares[agent_name] = square

# Thiết lập các thông số đồ thị
ax.set_title('Agent Positions on a 10x10 Grid')
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_xticks(range(grid_size + 1))
ax.set_yticks(range(grid_size + 1))
ax.set_xlim(0, grid_size)
ax.set_ylim(0, grid_size)
ax.legend()
ax.grid(True)

# Hàm cập nhật thông tin mới từ file YAML
def update_info_from_yaml():
    global steps
    with open('output.yaml', 'r') as file:
        new_data = yaml.safe_load(file)
        agent_name = 'agent3'  # Thay thế bằng tên của agent bạn quan tâm
        agent_schedule = data['schedule'][agent_name]
        steps = len(agent_schedule)
    
    for agent_name, agent_data in new_data['schedule'].items():
        agents[agent_name]['path'] = agent_data
        agents[agent_name]['position'] = {'x': agent_data[0]['x'], 'y': agent_data[0]['y']}

# Hàm cập nhật dữ liệu
def update(frame):
    global agents
    
    # Cập nhật vị trí mới cho các agent
    for agent_name, agent_data in agents.items():
        if frame < len(agent_data['path']):
            next_position = agent_data['path'][frame]
            agents[agent_name]['position'] = {'x': next_position['x'], 'y': next_position['y']}
            
            # Cập nhật vị trí của hình vuông đại diện cho agent
            agent_squares[agent_name].set_xy((next_position['x'] - 0.5, next_position['y'] - 0.5))

    # Hiển thị thông tin hoặc thực hiện các thao tác khác tại thời điểm hiện tại
    print(f'Time: {frame}')
    for agent_name, agent_data in agents.items():
        print(f"{agent_name}: ({agent_data['position']['x']}, {agent_data['position']['y']})")

    # Cập nhật thông tin mới từ file YAML nếu cần
    if frame == steps-1:  # Ví dụ: cập nhật thông tin mỗi 3 bước thời gian
        print(steps)
        print(frame)
        update_info_from_yaml()

# Tạo animation
animation = FuncAnimation(fig, update, frames=range(steps), interval=500)

# Hiển thị đồ thị
plt.show()
