#!/usr/bin/python
import argparse
import glob
from pathlib import Path

from visualize_demo import Animation
from single_agent_planner import get_sum_of_cost

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run a simple MAPF visualization without CBS')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    args = parser.parse_args()
    if args.batch:
        # batch 모드인 경우: test 파일 여러 개 실행
        input_instance = ["D:/git/MAPF-ICBS/code/instances/map.txt"]
    elif args.instance is not None:
        # instance 파일 인자가 있을 경우
        input_instance = sorted(glob.glob(args.instance))
    else:
        # 아무 인자도 없을 경우 기본값 지정 or 예외 처리
        print("No instance file specified. Using default: instances/map.txt")
        input_instance = ["D:/git/MAPF-ICBS/code/instances/map.txt"]

    for file in input_instance:
        print("***Import an instance***")
        print(file)
        my_map, starts, goals = import_mapf_instance(file)
        print_mapf_instance(my_map, starts, goals)

        # ---------------------
        # 1) 여기서 직접 paths를 작성하거나, 따로 계산해서 생성
        #    예시는 단순히 "에이전트가 시작지에서 목적지까지 직선으로 가정"한 샘플
        #    필요에 따라 직접 작성/수정하세요.
        # ---------------------
        paths = []
        path_agent0 = [(0,0),(0,0),(0,0),(0,0), (0,1), (0,2),(0,3),(0,4),(0,5),(0,6),(1,6),(2,6),(3,6),(4,6),(5,6),(6,6),(6,7),]
        paths.append(path_agent0)

        # 에이전트 1
        path_agent1 = [(2,0),(3,0),(4,0),(5,0),(6,0),(7,0),(8,0),(9,0),(10,0),(10,1),(10,2)]
        paths.append(path_agent1)
        
        path_agent2 = [(11,10),(11,10),(11,10),(11,10),(11,10),(11,10),(11,10),(11,9),(11,8),(11,7),(11,6), (10,6), (10,5),(9,5),(8,5),(7,5)]
        paths.append(path_agent2)

        cost = get_sum_of_cost(paths)

        # ---------------------
        # 2) 시각화
        # ---------------------
        offsets = [4, 0, 7]
        if not args.batch:
            print("***Visualize paths***")
            animation = Animation(my_map, starts, goals, paths, offsets=offsets)
            animation.show()
            animation.save("demo.gif", speed=1.0)
            # animation.save("output.mp4", 1.0)  # 원하면 저장
