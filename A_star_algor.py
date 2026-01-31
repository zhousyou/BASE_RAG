"""
A*算法实现
用于在网格地图中寻找从起点到终点的最短路径。
使用曼哈顿距离作为启发式函数。
"""

class Node:
    """
    表示网格中的一个节点
    """
    def __init__(self, position, parent=None):
        self.position = position  # 节点的位置，格式为 (行, 列)
        self.parent = parent      # 父节点，用于重构路径
        self.g = 0                # 从起点到当前节点的实际代价
        self.h = 0                # 启发式估计从当前节点到终点的代价
        self.f = 0                # 总代价 f = g + h

    def __eq__(self, other):
        return self.position == other.position


def heuristic(current, goal):
    """
    计算启发式函数（曼哈顿距离）
    :param current: 当前节点位置 (row, col)
    :param goal: 目标节点位置 (row, col)
    :return: 曼哈顿距离
    """
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

def a_star(grid, start, end):
    """
    A* 算法主函数
    :param grid: 二维列表，表示地图，0 表示可通过，1 表示障碍物
    :param start: 起始位置 (row, col)
    :param end: 终止位置 (row, col)
    :return: 成功找到路径则返回路径列表，否则返回 None
    """
    # 创建起点和终点节点
    start_node = Node(start)
    goal_node = Node(end)

    # 初始化开放列表和关闭列表
    open_list = []
    closed_list = []

    # 将起点加入开放列表
    open_list.append(start_node)

    # 四个方向移动：上、下、左、右
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    # 主循环
    while len(open_list) > 0:
        # 找出f值最小的节点
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # 将当前节点从开放列表移除，加入关闭列表
        open_list.pop(current_index)
        closed_list.append(current_node)

        # 检查是否已到达目标
        if current_node == goal_node:
            path = []
            while current_node is not None:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]  # 返回反转的路径（从起点到终点）

        # 生成子节点
        children = []<br>        for direction in directions:
            # 计算子节点位置
            node_position = (
                current_node.position[0] + direction[0],
                current_node.position[1] + direction[1]
            )

            # 确保位置在地图范围内
            if (
                node_position[0] < 0
                or node_position[0] >= len(grid)
                or node_position[1] < 0
                or node_position[1] >= len(grid[0])
            ):
                continue

            # 确保没有障碍物
            if grid[node_position[0]][node_position[1]] != 0:
                continue

            # 创建新的子节点
            new_node = Node(node_position, current_node)
            children.append(new_node)

        # 遍历所有子节点
        for child in children:
            # 如果子节点已经在关闭列表中，跳过
            if child in closed_list:
                continue

            # 计算g, h, f值
            child.g = current_node.g + 1
            child.h = heuristic(child.position, goal_node.position)
            child.f = child.g + child.h

            # 如果子节点已经在开放列表中，并且新的路径代价更高，则跳过
            if any(open_node for open_node in open_list if child == open_node and child.g > open_node.g):
                continue

            # 将子节点加入开放列表
            open_list.append(child)

    # 如果没有找到路径，返回None
    return None


# 示例使用代码
if __name__ == '__main__':
    # 创建一个简单的网格地图（0 表示空地，1 表示障碍）
    grid = [
        [0, 0, 0, 1, 0, 0, 0],
        [0, 1, 0, 1, 0, 1, 0],
        [0, 1, 0, 0, 0, 1, 0],
        [0, 0, 0, 1, 0, 0, 0],
        [0, 1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 1, 0],
        [0, 1, 1, 1, 1, 1, 0]
    ]

    start = (0, 0)  # 起始点
    end = (6, 6)    # 目标点

    path = a_star(grid, start, end)
    if path:
        print("找到路径:", path)
    else:
        print("未找到路径")