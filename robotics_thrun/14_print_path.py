grid =[
    [0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 1, 0],
    [0, 0, 1, 0, 1, 0],
    [0, 0, 1, 0, 1, 0]
]

init = [0,0]
goal = [len(grid)-1, len(grid[0]) -1]

cost = 1
delta =[
    [-1, 0], #go up
    [0, -1], #go left
    [1, 0 ], #go down
    [0 , 1] # gro right
]

delta_name = ["^", "<", "v" , ">"]

def search(grid, init, goal, cost):
    closed =[[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    expand = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
    action = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed[init[0]][init[1]] =1
    action[init[0]][init[1]] =1
    x = init[0]
    y = init[1]
    g = 0
    count = 0
    open = [[g, x, y]]
    found = False # flag that is set when search is complete
    resign = False # flag set if we can't find goal


    while found is False and resign is False:
        # no more open grid means that we can not find goal
        if len(open) == 0:
            resign = True
            return "fail"
        # len(open) != 0 means that searching goal is not completed
        else:
            #opens sort
            open.sort()
            #open revese sort -> minmum gvalue on top
            open.reverse()
            # get minimum gvalue open(minimum cost) 
            next = open.pop()

            x = next[1]
            y= next[2]
            g = next[0]
            expand[x][y] = count
            count +=1

            #arrived at goal
            if x == goal[0] and y ==goal[1]:
                found = True
            #not arrived yet
            else:
                # try all delta
                for i in range(len(delta)):
                    x2 = x+delta[i][0]
                    y2 = y + delta[i][1]

                    #check moved pose is in a grid
                    if x2 >=0 and x2<len(grid) and y2>=0 and y2<len(grid[0]):
                        #check current position is obstacle or not
                        #if current is not obstacle and not checked yet, add gvalue and append to open
                        if grid[x2][y2] == 0 and closed[x2][y2] ==0:
                            g2 = g+cost
                            open.append([g2, x2, y2])
                            closed[x2][y2] = 1
                            action[x2][y2] = i

    for i in range(len(expand)):
        print(expand[i])

    print()
    for i in range(len(action)):
        print(action[i])

    policy =[[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    x = goal[0]
    y =goal[1]
    policy[x][y] = '*'
    while x != init[0] or y != init[1]:
        #action[x][y] == 3 -> move right
        #action[x][y] == 2 -> move down
        #delta[3][0] -> down move delta x value
        #delta[3][1] -> down move delta y value

        #first goal xy and delta[action[x][y]] means down movement
        # x2,y2(3,5) = x,y(4,5) - down(1,0) -> x2, y2 up of x,y
        # next pose 3,5
        x2 = x - delta[action[x][y]][0]
        y2 = y - delta[action[x][y]][1]
        # policy pose is assigned delta_name[down] "v"
        policy[x2][y2] = delta_name[action[x][y]]
        x = x2
        y = y2

    print()
    for i in range(len(action)):
        print(policy[i])


search(grid,init,goal,cost)