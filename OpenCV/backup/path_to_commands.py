def direction_between(pos1, pos2):
    """두 좌표를 보고 방향을 계산."""
    dy = pos2[0] - pos1[0]
    dx = pos2[1] - pos1[1]
    if dy == -1 and dx == 0:
        return "north"
    elif dy == 1 and dx == 0:
        return "south"
    elif dy == 0 and dx == 1:
        return "east"
    elif dy == 0 and dx == -1:
        return "west"
    else:
        raise ValueError(f"Invalid move from {pos1} to {pos2}")

def compute_turns(current_dir, target_dir):
    """현재 방향과 목표 방향을 비교해 'left', 'right', 'forward'를 계산."""
    directions = ["north", "east", "south", "west"]
    idx_current = directions.index(current_dir)
    idx_target = directions.index(target_dir)

    diff = (idx_target - idx_current) % 4
    if diff == 0:
        return []
    elif diff == 1:
        return ["right"]
    elif diff == 3:
        return ["left"]
    elif diff == 2:
        return ["right", "right"]  # 180도 턴
    else:
        return []

def path_to_commands(path, initial_dir="north"):
    commands = []
    current_dir = initial_dir

    for curr_pos, next_pos in zip(path, path[1:]):
        if curr_pos == next_pos:
            # ✨ 현재 위치와 다음 위치가 같으면 'stop' 명령 추가
            commands.append("stop")
            continue

        target_dir = direction_between(curr_pos, next_pos)

        # 현재 방향과 목표 방향이 다르면 회전 명령 추가
        turn_cmds = turns_needed(current_dir, target_dir)
        commands.extend(turn_cmds)

        # 전진 명령 추가
        commands.append("forward")

        # 방향 업데이트
        current_dir = target_dir

    return commands

def turns_needed(current_dir, target_dir):
    dirs = ["north", "east", "south", "west"]
    idx_current = dirs.index(current_dir)
    idx_target = dirs.index(target_dir)

    diff = (idx_target - idx_current) % 4

    if diff == 0:
        return []
    elif diff == 1:
        return ["right"]
    elif diff == 2:
        return ["right", "right"]
    elif diff == 3:
        return ["left"]
    else:
        raise ValueError(f"Invalid turn from {current_dir} to {target_dir}")