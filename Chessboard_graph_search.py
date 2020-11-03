import numpy as np
from collections import deque

def main():
    stateX = range(1,9)                                         #X,Y coordinates on a 8x8 chessboard
    stateY = range(1,9)
    state = [stateX, stateY]
    actions = [(-2,1), (-2,-1), (-1,2), (-1,-2), (1,2), (1,-2), (2,1),(2,-1)]  #actions of a knight on a chessboard
    start = (1,1)                                                        # choose any start state
    goal = (8,8)                                                         # choose any goal state
    V,E = generateGraph(state, actions)
    path = bfs(V, E, state, start, goal)
    print(path)
    path = dfs(V, E, state, start, goal)
    print(path)


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

def bfs(V, E, state, start, goal):
    q = deque()
    pred = []
    visited = np.zeros((len(state[0]), len(state[1])))
    q.append(start)
    visited[start[0]-1][start[1]-1] = 1
    while q:
        vertex = q.popleft()
        for edge in E:
            if edge[0] == vertex and visited[edge[1][0]-1][edge[1][1]-1]==0:
                q.append(edge[1])
                pred.append((edge[1], edge[0]))
                visited[edge[1][0] - 1][edge[1][1] - 1] = 1
                if edge[1] == goal:
                    break;
    path = []
    path.append(goal)
    while path[-1] != start:
        for node in pred:
            if node[0] == path[-1]:
                path.append(node[1])
    return path

def dfs(V, E, state, start, goal):
    q = deque()
    pred = []
    visited = np.zeros((len(state[0]), len(state[1])))
    q.append(start)
    visited[start[0]-1][start[1]-1] = 1
    while q:
        vertex = q.pop()
        for edge in E:
            if edge[0] == vertex and visited[edge[1][0]-1][edge[1][1]-1]==0:
                q.append(edge[1])
                pred.append((edge[1], edge[0]))
                visited[edge[1][0] - 1][edge[1][1] - 1] = 1
                if edge[1] == goal:
                    break;
    path = []
    path.append(goal)
    while path[-1] != start:
        for node in pred:
            if node[0] == path[-1]:
                path.append(node[1])
    return path

main()
