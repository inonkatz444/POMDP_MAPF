from functools import reduce
from random import sample, seed
import numpy as np
import sys
from transitions_generator import generate_transitions

def get_loc(pos: int, rows: int, columns: int) -> tuple[int, int]:
    return pos // columns, pos % columns

def get_neighbors(loc: tuple[int, int], grid: np.ndarray) -> list[tuple[int, int]]:
    neighbors = []
    if loc[0] > 0 and grid[loc[0] - 1, loc[1]] != "#":
        neighbors.append((loc[0] - 1, loc[1]))
    if loc[0] < grid.shape[0]-1 and grid[loc[0] + 1, loc[1]] != "#":
        neighbors.append((loc[0] + 1, loc[1]))
    if loc[1] > 0 and grid[loc[0], loc[1] - 1] != "#":
        neighbors.append((loc[0], loc[1] - 1))
    if loc[1] < grid.shape[1]-1 and grid[loc[0], loc[1] + 1] != "#":
        neighbors.append((loc[0], loc[1] + 1))

    return neighbors

def valid_grid(grid: np.ndarray, agent_start: int, agent_end: int) -> bool:
    visited = np.full(grid.shape, False)
    rows = grid.shape[0]
    columns = grid.shape[1]
    open_list = [get_loc(agent_start, rows, columns)]
    goal = get_loc(agent_end, rows, columns)
    while len(open_list) > 0:
        node = open_list.pop()
        if node == goal:
            return True

        visited[node] = True
        for loc in get_neighbors(node, grid):
            if not visited[loc]:
                open_list.append(loc)
    return False

def generate_env(rows: int, columns: int, num_of_holes: int) -> str:
    agent1_start = columns + 1
    agent2_start = 2*columns - 2
    agent1_end = (rows-1)*columns - 2
    agent2_end = (rows-2)*columns + 1
    beacon_pos = ((rows+1)//2) * columns - 2

    samples_range = list(range(rows*columns))
    samples_range.remove(agent1_start)
    samples_range.remove(agent1_end)
    samples_range.remove(agent2_start)
    samples_range.remove(agent2_end)
    if samples_range.__contains__(beacon_pos):
        samples_range.remove(beacon_pos)

    attempts = 0

    while (True):
        holes = sample(samples_range, num_of_holes)
        holes.sort()
        attempts = attempts + 1

        grid = np.full(rows*columns, " ")
        grid[holes] = "#"
        grid[agent1_start] = "a"
        grid[agent1_end] = "A"
        grid[agent2_start] = "b"
        grid[agent2_end] = "B"
        beacon_range = ((rows+columns) / 2) // 2
        if grid[beacon_pos] == " ":
            grid[beacon_pos] = str(beacon_range)
        grid = grid.reshape((rows, columns))
        print(grid)

        if valid_grid(grid, agent1_start, agent1_end) and valid_grid(grid, agent2_start, agent2_end):
            break
        else:
            print("\nnot valid grid!")

        if attempts >= 10000:
            print("TIMEOUT!")
            return "", [], -1, -1


    env = "#"*(columns+2) + "\n"
    for i in range(rows):
        env += "#"
        env += reduce(lambda s, c: s + c, grid[i, :], "")
        env += "#\n"
    env += "#"*(columns+2)
    return env, agent1_start, agent2_start ,agent1_end, agent2_end, holes, beacon_pos, beacon_range

if __name__ == "__main__":
    if len(sys.argv) == 4:
        rows = int(sys.argv[1])
        columns = int(sys.argv[2])
        total_num_of_holes = int(sys.argv[3])
        if (rows >= 4 and columns >= 4):
            if total_num_of_holes <= rows*columns-5:
                env, agent1_start, agent2_start ,agent1_end, agent2_end, holes, beacon_pos, beacon_range = generate_env(rows, columns, total_num_of_holes)

                if env == "":
                    print("couldn't generate environment")
                    exit(0)
                else:
                    print(env)

                def num_of_holes(min_place: int, max_place: int) -> int:
                    count = 0
                    i = 0
                    while i < len(holes) and holes[i] < min_place:
                        i += 1
                    while i < len(holes) and holes[i] <= max_place:
                        count += 1
                        i +=1

                    return count

                agent1_start = agent1_start - num_of_holes(0, agent1_start)
                agent1_end = agent1_end - num_of_holes(0, agent1_end)
                agent2_start = agent2_start - num_of_holes(0, agent2_start)
                agent2_end = agent2_end - num_of_holes(0, agent2_end)

                with open(f"open_world_{rows}_{columns}_{total_num_of_holes}.POMDP", "w") as model:
                    model.write(f"# file_name: open_world_{rows}_{columns}_{total_num_of_holes}\n\n")
                    model.write(f"# A randomly-generated open-world problem with 2 agents a, b, with {total_num_of_holes} walls\n\n")
                    model.write("# The maze looks like this:\n#   <num>: Beacon with influence range of num, <lower-case letter>: start position of <letter>, <upper-case letter>: end position of <letter> - positive\n\n")

                    model.write("\n".join(map(lambda s: "#   " + s, env.split("\n"))) + "\n\n")
                    model.write("""# The actions, NSEW, have the expected result 80% of the time, and
# transition in a direction perpendicular to the intended on with a 10%
# probability for each direction. Movement into a wall returns the agent
# to its original state.\n\n""")
                    model.write(f"rows: {rows}\n")
                    model.write(f"cols: {columns}\n")
                    model.write("discount: 0.99\n")
                    model.write("values: reward\n")
                    model.write(f"states: {rows*columns-total_num_of_holes}\n")
                    model.write(f"actions: n s e w noop\n\n")

                    model.write("start_states:\n")
                    model.write(f"a {agent1_start}\n")
                    model.write(f"b {agent2_start}\n\n")

                    model.write("end_states:\n")
                    model.write(f"a {agent1_end}\n")
                    model.write(f"b {agent2_end}\n\n")

                    model.write("holes:\n")
                    for hole in holes:
                        hole_loc = get_loc(hole, rows, columns)
                        model.write(f"{hole_loc[0]} {hole_loc[1]}\n")

                    beacon_loc = get_loc(beacon_pos, rows, columns)
                    model.write("\nbeacons:\n")
                    model.write(f"{beacon_loc[0]} {beacon_loc[1]} : {int(beacon_range)}\n\n")
                    model.write(generate_transitions(env))
            else:
                print("too many holes!")
        else:
            print("rows and columns must be greater then 3")
    else:
        print("usage: python3 " + __file__ + " <rows> <columns> <total_num_of_holes>")