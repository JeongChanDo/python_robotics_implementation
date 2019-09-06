# -----------
# User Instructions:
# 
# Modify the function search so that it returns
# a table of values called expand. This table
# will keep track of which step each node was
# expanded.
#
# Make sure that the initial cell in the grid 
# you return has the value 0.
# ----------

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid,init,goal,cost):
    # ----------------------------------------
    # modify code below
    # ----------------------------------------
    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid[:]))]
    closed[init[0]][init[1]] =1
    expand = [[-1 for row in range(len(grid[0]))] for col in range(len(grid[:]))]
    
    x = init[0]
    y = init[1]
    g = 0
    open = [[g, x, y]]

    #flag that is set when search complete
    found = False
    #flag set if we can't find expand
    resign = False
    #count expansion
    count = 0

    while found is False and resign is False:
        #check if we still have eleemtns on open list
        if len(open) == 0:
            resign = True
            print("fail")

        else:
            """
            1st step
            0, 0, 0
            2nd step
            1, 0, 1    0, 0, 0(smallest gval)
            3rd step 
            1, 0, 1    1, 1, 0  

            """
            #remove node from list
            open.sort()
            open.reverse()
            #print("len current pose : ",len(open), "   open : ",open)
            #get smallist gvalue
            next = open.pop()
            #print("smallest value : ",next)

            x = next[1]
            y = next[2]
            g = next[0]
            expand[x][y] = count
            count +=1


            #check if we are done
            if x == goal[0] and y==goal[1]:
                found = True
                #print(next)
                #print("###Search successful")
            else:
                #expand wining element and add to new open list
                for i in range(len(delta)):
                    #move robot
                    x2 = x + delta[i][0]
                    y2 = y +delta[i][1]
                    #robot in grid
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2< len(grid[0]):
                        #not closed grid(not yet checked) and not occupied(no obstacles)
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            #gvalue up
                            g2 = g+cost
                            #new open grid
                            open.append([g2, x2, y2])
                            #check current grid
                            closed[x2][y2] = 1
                            
    for i in range(len(expand)):
        print(expand[i])


search(grid,init,goal,cost)