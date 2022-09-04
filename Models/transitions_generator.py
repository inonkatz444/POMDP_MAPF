def generate_transitions(env: str):
    lines = env.split('\n')

    rows = len(lines) - 2
    cols = len(lines[0]) - 2

    holes = []

    for i in range(1, len(lines)-1):
        row = lines[i][1:-1]
        for j in range(cols):
            if row[j] == "#":
                holes.append((i-1)*cols+j)

    def is_hole(iplace: int) -> bool:
        return iplace in holes

    def has_self_trans(prob: float) -> bool:
        return prob != 0.0

    def in_left_bounds(iplace: int) -> bool:
        return iplace % cols == 0

    def in_right_bounds(iplace: int) -> bool:
        return iplace % cols == cols - 1

    def in_up_bounds(iplace: int) -> bool:
        return iplace < cols

    def in_down_bounds(iplace: int) -> bool:
        return iplace >= (rows*cols) - cols

    def num_of_holes(min_place: int, max_place: int) -> int:
        count = 0
        i = 0
        while i < len(holes) and holes[i] < min_place:
            i += 1
        while i < len(holes) and holes[i] <= max_place:
            count += 1
            i +=1

        return count

    transitions = ""

    #north transitions
    istate = 0
    iplace = 0
    prob = 0.0

    while iplace < cols*rows:
        if not is_hole(iplace):
            prob = 0.0
            if in_up_bounds(iplace) or is_hole(iplace-cols):
                prob += 0.8
            else:
                holes_between = num_of_holes(iplace-cols, iplace)
                transitions += f"T: n   : {istate}          : {istate-cols+holes_between}        0.8\n"
            if in_left_bounds(iplace) or is_hole(iplace-1):
                prob += 0.1
            else:
                transitions += f"T: n   : {istate}          : {istate-1}        0.1\n"
            if in_right_bounds(iplace) or is_hole(iplace+1):
                prob += 0.1
            else:
                if has_self_trans(prob):       # for pretty printings
                    transitions += f"T: n   : {istate}          : {istate+1}        0.1\n"
                else:
                    transitions += f"T: n   : {istate}          : {istate+1}        0.1\n\n"
            if has_self_trans(prob):
                transitions += f"T: n   : {istate}          : {istate}        {prob}\n\n"
            istate += 1
        iplace += 1

    transitions += "\n\n"


    #south transitions
    istate = 0
    iplace = 0
    prob = 0.0
    while iplace < cols*rows:
        if not is_hole(iplace):
            prob = 0.0
            if in_down_bounds(iplace) or is_hole(iplace+cols):
                prob += 0.8
            else:
                holes_between = num_of_holes(iplace, iplace+cols)
                transitions += f"T: s   : {istate}          : {istate+cols-holes_between}        0.8\n"
            if in_left_bounds(iplace) or is_hole(iplace-1):
                prob += 0.1
            else:
                transitions += f"T: s   : {istate}          : {istate-1}        0.1\n"
            if in_right_bounds(iplace) or is_hole(iplace+1):
                prob += 0.1
            else:
                if has_self_trans(prob):       # for pretty printings
                    transitions += f"T: s   : {istate}          : {istate+1}        0.1\n"
                else:
                    transitions += f"T: s   : {istate}          : {istate+1}        0.1\n\n"
            if has_self_trans(prob):
                transitions += f"T: s   : {istate}          : {istate}        {prob}\n\n"
            istate += 1
        iplace += 1

    transitions += "\n\n"

    #east transitions
    istate = 0
    iplace = 0
    prob = 0.0
    while iplace < cols*rows:
        if not is_hole(iplace):
            prob = 0.0
            if in_right_bounds(iplace) or is_hole(iplace+1):
                prob += 0.8
            else:
                transitions += f"T: e   : {istate}          : {istate+1}        0.8\n"
            if in_up_bounds(iplace) or is_hole(iplace-cols):
                prob += 0.1
            else:
                holes_between = num_of_holes(iplace-cols, iplace)
                transitions += f"T: e   : {istate}          : {istate-cols+holes_between}        0.1\n"
            if in_down_bounds(iplace) or is_hole(iplace+cols):
                prob += 0.1
            else:
                holes_between = num_of_holes(iplace, iplace+cols)
                if has_self_trans(prob):       # for pretty printings
                    transitions += f"T: e   : {istate}          : {istate+cols-holes_between}        0.1\n"
                else:
                    transitions += f"T: e   : {istate}          : {istate+cols-holes_between}        0.1\n\n"
            if has_self_trans(prob):
                transitions += f"T: e   : {istate}          : {istate}        {prob}\n\n"
            istate += 1
        iplace += 1
    
    transitions += "\n\n"

    #west transitions
    istate = 0
    iplace = 0
    prob = 0.0
    while iplace < cols*rows:
        if not is_hole(iplace):
            prob = 0.0
            if in_left_bounds(iplace) or is_hole(iplace-1):
                prob += 0.8
            else:
                transitions += f"T: w   : {istate}          : {istate-1}        0.8\n"
            if in_up_bounds(iplace) or is_hole(iplace-cols):
                prob += 0.1
            else:
                holes_between = num_of_holes(iplace-cols, iplace)
                transitions += f"T: w   : {istate}          : {istate-cols+holes_between}        0.1\n"
            if in_down_bounds(iplace) or is_hole(iplace+cols):
                prob += 0.1
            else:
                holes_between = num_of_holes(iplace, iplace+cols)
                if has_self_trans(prob):       # for pretty printings
                    transitions += f"T: w   : {istate}          : {istate+cols-holes_between}        0.1\n"
                else:
                    transitions += f"T: w   : {istate}          : {istate+cols-holes_between}        0.1\n\n"
            if has_self_trans(prob):
                transitions += f"T: w   : {istate}          : {istate}        {prob}\n\n"
            istate += 1
        iplace += 1
    transitions += "\n\n"

    # noop transitions

    for istate in range(rows*cols - len(holes)):
        transitions += f"T: noop   : {istate}          : {istate}        1.0\n"

    return transitions

env = """#######
#     #
#     #
##   ##
#     #
#     #
#######"""

transitions = generate_transitions(env)
with open('transitions', "w") as transitions_file:
    transitions_file.write(transitions)