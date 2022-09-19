def count_collisions(file):
    count = 0
    while True:
        line = file.readline()
        if not line:
            break

        if "collide" in line:
            count += 1
            while "collide" in line:
                line = file.readline()
    return count

if __name__ == "__main__":
    with open("easy_5_FSVI_track.txt", "r") as f:
        print(count_collisions(f))