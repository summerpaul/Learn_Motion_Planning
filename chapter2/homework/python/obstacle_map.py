import random
def obstacle_map(xStart,yStart,xTarget,yTarget,MAX_X,MAX_Y):
    rand_map = random(MAX_X, MAX_Y)
    map = []
    map[1, 1] = xStart
    map[1, 2] = yStart
    k=2
    obstacle_ratio = 0.25
    for i in range(MAX_X):
        for j in range(MAX_Y):
            if((rand_map[i, j] < obstacle_ratio) & (i != xStart | j != yStart) & (i != xTarget | j != yTarget)):
                map[k, 1] = i
                map[k, 2] = j
                k = k + 1
    map[k, 1] = xTarget
    map[k, 2] = yTarget
    return map 