import numpy as np
from collections import deque
import matplotlib.pyplot as plt

def main():
    stateX = range(0,100)
    stateY = range(0,100)
    state = [stateX, stateY]
    actions = [(1,1), (0,1), (1,0), (-1,-1), (-1,0), (0,-1), (-1,1), (1,-1)]
    start = (1,1)                                                        # choose any start state
    goal = (99,99)                                                         # choose any goal state within the state space
    numObs = 30								#number of obstacles
    distance = 10							#distance threshold for PRM
    numPoints = 1000
    cObs = createObstacles(state, numObs)
    [V,E, state_bar] = PRMGraph(start, goal, cObs, state, numPoints, distance, actions)          #state bar the smaller state space created by random sampling, similarly for V and E
    
    path = bfs(V, E, state_bar, goal, start, cObs)
    print(path)

    x_val = [x[0][0] for x in path]						#plot path, obstacles
    y_val = [x[0][1] for x in path]
    plt.plot(x_val,y_val,'or')
    x_val = [x[0] for x in cObs]
    y_val = [x[1] for x in cObs]
    plt.plot(x_val,y_val,'xb')
    plt.show()



def bfs(V, E, state, start, goal, cObs):
    if  len(state[0])==1 and len(state[1])==1:				#modified bfs traversal that returns a series of edges 
        return []
    q = deque()
    pred = []
    path = []
    visited = np.zeros((len(state[0]), len(state[1])))
    q.append(start)
    visited[0][0] = 1
    while q:
        vertex = q.popleft()
        for edge in E:
            if edge[0] == vertex and visited[edge[1][0]-start[0]][edge[1][1]-start[1]]==0 and edge[1] not in cObs:
                q.append(edge[1])
                pred.append((edge[1], edge[0]))
                visited[edge[1][0] - start[0]][edge[1][1] - start[1]] = 1
                if edge[1] == goal:
                    path.append((edge[1], edge[0]))
                    break;
    #path.append(goal)
    #print(pred)
    #print(path)
    if not path:
        return []
    while path[-1][1] != start:
        for node in pred:
            if node[0] == path[-1][1]:
                path.append(node)
    return path

#creates numObs number of obstacles in the state space
def createObstacles(state, numObs):
    cObs = []                                                   # obstacles in config state space
    for i in range(numObs):
        randx = np.random.randint(10, len(state[0])-10)
        randy = np.random.randint(10, len(state[1])-10)
        obsLen = np.random.randint(3, 20)
        obsThick = np.random.randint(2, 10)
        obsOrient = np.random.randint(0, 1)
        if obsOrient == 0:                                       #horizontal rectangle
            for i in range(obsLen):
                for j in range(obsThick):
                    cObs.append((randx+i, randy+j))
        if obsOrient == 1:                                       #vertical rectangle
            for i in range(obsLen):
                for j in range(obsThick):
                    cObs.append((randx+j, randy+i))
    return cObs


def generateGraph(state, actions):
    V = []
    E = []
    for i in state[0]:
        for j in state[1]:
            V.append((i,j))
            for k in actions:
                iNew = i+k[0]
                jNew = j+k[1]
                if (iNew in state[0]) and (jNew in state[1]):
                    E.append(((i, j), (iNew, jNew)))
    return V,E


def PRMGraph(start, goal, cObs, state, numPoints, distance, actions):
    state_bar = [[],[]]
    V = [start]
    E = []
    for n in range(numPoints):
        randx = np.random.randint(0, len(state[0]))       #pick random number from 0 to 99 inclusive
        randy = np.random.randint(0, len(state[1]))		#PRM algorithm
        if (randx, randy) not in cObs:
            ct = 0
            for v in V:
                dist = np.sqrt((randx-v[0])**2 + (randy-v[1])**2)
                if dist <= distance and ct<9:
                    ct = ct+1
                    maxx = max(v[0], randx)
                    maxy = max(v[1], randy)
                    minx = min(v[0], randx)
                    miny = min(v[1], randy)
                    stateSub = [state[0][minx:maxx+1], state[1][miny:maxy+1]]
                    #print(stateSub)
                    [vSub, eSub] = generateGraph(stateSub, actions)
                    bfsPath = bfs(vSub, eSub, stateSub, v, (randx, randy), cObs)
                    if bfsPath:
                        E = E + bfsPath
            V.append((randx, randy))
            state_bar[0].append(randx)
            state_bar[1].append(randy)

    # add the goal at the end to ensure there is path to the goal
    for v in V:
        dist = np.sqrt((goal[0]-v[0])**2 + (goal[1]-v[1])**2)
        if dist <= distance*3:
            maxx = max(v[0], goal[0])
            maxy = max(v[1], goal[1])
            minx = min(v[0], goal[0])
            miny = min(v[1], goal[1])
            stateSub = [state[0][minx:maxx+1], state[1][miny:maxy+1]]
            [vSub, eSub] = generateGraph(stateSub, actions)
            bfsPath = bfs(vSub, eSub, stateSub, v, goal, cObs)
            #print(bfsPath)
            if bfsPath:
                E = E + bfsPath
    V.append(goal)
    state_bar[0].append(goal[0])
    state_bar[1].append(goal[1])
    return V,E, state_bar





main()
