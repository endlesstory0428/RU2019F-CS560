import sys

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np


def getDist(p, q):
    px, py = p
    qx, qy = q
    return np.sqrt(np.square(px - qx) + np.square(py - qy))

def getDet(p, v, n):
    px, py = p
    vx, vy = v
    nx, ny = n
    return (vx - nx) * (py - vy) - (px - vx) * (vy - ny)

def getDet4(a, b, dx, dy):
    ax, ay = a
    bx, by = b
    return (bx - ax) * dy - dx * (by - ay)

def getDot(p, v, n):
    px, py = p
    vx, vy = v
    nx, ny = n
    return (px - vx) * (nx - vx) + (py - vy) * (ny - vy)

def getLineFunc(p, q):
    px, py = p
    qx, qy = q
    def lineFunc(point):
        x, y = point
        return (x - px) * (qy - py) - (y - py) * (qx - px)
    return lineFunc

def getFoot(a, b, v):
    ax, ay = a
    bx, by = b
    vx, vy = v
    dx = ax - bx
    dy = ay - by

    r = ((vx - ax) * dx + (vy - ay) * dy) / (dx * dx + dy * dy)
    return (ax + r * dx, ay + r * dy)


class vertex(object):
    def __init__(self, idx, pos):
        super(vertex, self).__init__()
        self.idx = idx
        self.pos = pos
        self.adj = set()
        return

    def __hash__(self):
        return hash(self.pos)

    def __str__(self):
        return str(self.idx) + ': ' + str(self.pos)

    def __repr__(self):
        return str(self)

    def incIdx(self):
        self.idx = self.idx + 1

    def checkCollision(self, obstacles = None, robot = None):
        if obstacles is None or robot is None:
            return False
        return not isCollisionFree(robot, self.pos, obstacles)


class edge(object):
    def __init__(self, u, v):
        super(edge, self).__init__()
        self.u = u
        self.v = v
        return
    
    def __hash__(self):
        return hash((self.u.pos, self.v.pos))

    # def _getUEdgedRobot(self, robot):
    #   uEdgedRobot = []
    #   robotSize = len(robot)
    #   dx = self.v.pos[0] - self.u.pos[0]
    #   dy = self.v.pos[1] - self.u.pos[0]
    #   nRobot = [(x + dx, y + dy) for x, y in robot]
    #   detList = [getDet4(robot[pIdx], robot[pIdx + 1], dx, dy) for pIdx in range (-1, robotSize - 1)]
    #   for pIdx in range(robotSize):
    #       nIdx = (pIdx + 1) % robotSize
    #       pDet = detList[pIdx]
    #       nDet = detList[nIdx]
    #       if pDet < 0:
    #           uEdgedRobot.append(robot[pIdx])
    #           if nDet > 0:
    #               uEdgedRobot.append(nRobot[pIdx])
    #       elif pDet > 0:
    #           uEdgedRobot.append(nRobot[pIdx])
    #           if nDet < 0:
    #               uEdgedRobot.append(robot[pIdx])
    #       else:
    #           if nDet > 0:
    #               uEdgedRobot.append(nRobot[pIdx])
    #           elif nDet < 0:
    #               uEdgedRobot.append(robot[pIdx])

    #   return uEdgedRobot

    # def checkCollision(self, obstacles = None, robot = None):
    #   if obstacles is None or robot is None:
    #       return False
    #   uEdgedRobot = self._getUEdgedRobot(robot)
    #   return not isCollisionFree(uEdgedRobot, self.u.pos, obstacles)
    def checkCollision(self, obstacles = None, robot = None):
        if obstacles is None or robot is None:
            return False
        npRobot = np.array(robot)
        dx = self.v.pos[0] - self.u.pos[0]
        dy = self.v.pos[1] - self.u.pos[1]
        stepSize = 0.25 * np.sqrt(np.sum(np.square(np.max(npRobot, axis = 0) - np.min(npRobot, axis = 0))))
        stepNum = int((np.sqrt(np.sum(np.square((dx, dy))))) // stepSize)
        if stepNum < 2:
            return False
        for i in range(1, stepNum):
            tempRobot = [(x + i * stepSize * dx, y + i * stepSize * dy) for x, y in robot]
            if not isCollisionFree(tempRobot, self.u.pos, obstacles):
                return True
        return False




class Tree(object):
    def __init__(self, initV):
        super(Tree, self).__init__()
        self.initV = dict() if initV is None else initV
        self.initVSize = len(self.initV)
        self.V = set()
        self.E = set()
        self.VIdx = 1
        return

    def _getNearestV(self, v):
        minV = None
        minDist = float('inf')
        for tempV in self.V:
            tempDist = getDist(v.pos, tempV.pos)
            if tempDist < minDist:
                minV = tempV
                minDist = tempDist
        return minV, minDist

    def _getNearestE(self, v):
        minE = None
        minDist = float('inf')
        minFoot = None
        for tempE in self.E:
            if getDot(tempE.u.pos, tempE.v.pos, v.pos) < 0:
                continue
            if getDot(tempE.v.pos, tempE.u.pos, v.pos) < 0:
                continue
            tempFoot = getFoot(tempE.u.pos, tempE.v.pos, v.pos)
            tempDist = getDist(v.pos, tempFoot)
            if tempDist < minDist:
                minE = tempE
                minDist = tempDist
                minFoot = tempFoot
        return minE, minDist, minFoot

    def grow(self, obstacles = None, robot = None):
        if obstacles is None:
            obstacles = []
        for idx, pos in self.initV.items():
            v = vertex(self.VIdx, pos)
            if v.checkCollision(obstacles, robot):
                #collision, pass
                continue
            #no collision, process this vertex
            if not self.V:
                #empty V, set root
                self.V.add(v)
                self.VIdx = self.VIdx + 1
            else:
                #grow tree
                vCand, vDist = self._getNearestV(v)
                eCand, eDist, candPos = self._getNearestE(v)
                if vDist <= eDist:
                    self._growV(v, vCand, obstacles, robot)
                else:
                    v.incIdx()
                    self._growE(v, eCand, candPos, obstacles, robot)
        return

    def extend(self, points, obstacles = None, robot = None):
        if obstacles is None:
            obstacles = []
        for pos in pointsDict:
            v = vertex(self.VIdx, pos)
            if v.checkCollision(obstacles, robot):
                #collision, pass
                continue
            #no collision, process this vertex
            if not self.V:
                #empty V, set root
                self.V.add(v)
                self.VIdx = self.VIdx + 1
            else:
                #grow tree
                vCand, vDist = self._getNearestV(v)
                eCand, eDist, candPos = self._getNearestE(v)

                if vDist <= eDist:
                    self._growV(v, vCand, obstacles, robot)
                else:
                    self._growE(v, eCand, candPos, obstacles, robot)
        return

    def _growV(self, v, vCand, obstacles, robot):
        e = edge(v, vCand)
        if e.checkCollision(obstacles, robot):
            #collision, pass
            return
        #no collision, add v and e to tree
        self.V.add(v)
        self.E.add(e)
        v.adj.add(vCand)
        vCand.adj.add(v)
        self.VIdx = self.VIdx + 1
        return

    def _splitEdge(self, eCand, vCand):
        self.V.add(vCand)
        self.E.remove(eCand)
        self.E.add(edge(eCand.u, vCand))
        self.E.add(edge(vCand, eCand.v))
        eCand.u.adj.remove(eCand.v)
        eCand.u.adj.add(vCand)
        eCand.v.adj.remove(eCand.u)
        eCand.v.adj.add(vCand)
        vCand.adj.add(eCand.u)
        vCand.adj.add(eCand.v)
        return

    def _growE(self, v, eCand, candPos, obstacles, robot):
        vCand = vertex(self.VIdx, candPos)
        e = edge(v, vCand)
        if e.checkCollision(obstacles, robot):
            #collision, pass
            return
        #no collision, split eCand, add v and e to tree
        self._splitEdge(eCand, vCand)
        self.V.add(v)
        self.E.add(e)
        v.adj.add(vCand)
        vCand.adj.add(v)
        self.VIdx = self.VIdx + 2
        return

    def getDict(self):
        vDict = dict()
        eDict = dict()
        for v in self.V:
            vDict[v.idx] = v.pos
            eDict[v.idx] = sorted([u.idx for u in v.adj])
        return vDict, eDict
        

class BFS(object):
    def __init__(self, adjList):
        super(BFS, self).__init__()
        self.adjDict = adjList
        return

    def search(self, s, g):
        preDict = dict()
        fringe = [s]
        while fringe:
            tempV = fringe.pop(0)
            if tempV == g:
                path = self.getPath(preDict, s, g)
                return path
            for nextV in self.adjDict[tempV]:
                if nextV in preDict:
                    continue
                preDict[nextV] = tempV
                fringe.append(nextV)
        return []

    def getPath(self, preDict, s, g):
        path = []
        tempV = g
        while tempV != s:
            path.append(tempV)
            tempV = preDict[tempV]
        path.append(s)
        path.reverse()
        return path


class obstacle(object):
    """docstring for obstacle"""
    def __init__(self, V):
        super(obstacle, self).__init__()
        self.V = V
        self.VSize = len(V)
        self.convex = self.checkConvex()
        if self.convex:
            self.lineFuncList = self.getLineFuncList()
        return

    def __str__(self):
        return repr(self)

    def __repr__(self):
        if not self.convex:
            rtvl = repr([repr(c) for c in self.child])
        else:
            rtvl = repr(self.V)
        return rtvl

    def getLineFuncList(self):
        lineFuncList = []
        for pIdx in range(-1, self.VSize - 1):
            lineFuncList.append(getLineFunc(self.V[pIdx], self.V[pIdx+1]))
        return lineFuncList

    def _getDiagVertex(self, vIdx, pvLineFunc, negSign):
        for i in range(2, self.VSize - 1):
            tempIdx = (vIdx + i) % self.VSize
            if (pvLineFunc(self.V[tempIdx]) <= 0) is not negSign:
                return tempIdx
        print('E: fail to get diag vertex')
        exit(-1)

    def _splitV(self, vIdx, diagIdx):
        vIdx, diagIdx = sorted([vIdx, diagIdx])
        lV = self.V[: vIdx + 1] + self.V[diagIdx: ]
        rV = self.V[vIdx: diagIdx + 1]
        return lV, rV

    def checkConvex(self):
        for vIdx in range(self.VSize):
            pIdx = (vIdx - 1) % self.VSize
            nIdx = (vIdx + 1) % self.VSize
            #p->v->n
            if getDet(self.V[pIdx], self.V[vIdx], self.V[nIdx]) >= 0:
                #right turn
                continue
            #non-convex vertex, split
            pvLineFunc = getLineFunc(self.V[pIdx], self.V[vIdx])
            negSign = pvLineFunc(self.V[nIdx]) <= 0
            diagIdx = self._getDiagVertex(vIdx, pvLineFunc, negSign)
            lV, rV = self._splitV(vIdx, diagIdx)
            self.child = [obstacle(lV), obstacle(rV)]
            return False
        return True

    def _checkCollisionConvexV(self, robot):
        #check if robot in obstacle
        for point in robot:
            for vIdx in range(-1, self.VSize - 1):
                v = self.V[vIdx]
                n = self.V[vIdx + 1]
                if getDet(point, v, n) <= 0:
                    #left turn, not inside, check next point
                    break
            else:
                #all right turn, inside, collision
                return True
        #all points not inside, safe
        #check if obstacle in robot
        for point in self.V:
            for vIdx in range(-1, len(robot) - 1):
                v = robot[vIdx]
                n = robot[vIdx + 1]
                if getDet(point, v, n) <= 0:
                    break
            else:
                return True
        #vertex safe
        return False

    def _checkCollisionConvexE(self, robot, robotFuncList):
        for i in range(len(robotFuncList)):
            robotFunc = robotFuncList[i]
            pr = robot[i - 1]
            vr = robot[i]
            for j in range(len(self.lineFuncList)):
                lineFunc = self.lineFuncList[j]
                po = self.V[j - 1]
                vo = self.V[j]
                if robotFunc(po) * robotFunc(vo) < 0 and lineFunc(pr) * lineFunc(vr) < 0:
                    return True
        return False


    def _checkCollisionConvex(self, robot, robotFuncList):
        if self._checkCollisionConvexV(robot):
            return True
        if self._checkCollisionConvexE(robot, robotFuncList):
            return True
        return False

    def checkCollision(self, robot):
        robotFuncList = [getLineFunc(robot[pIdx], robot[pIdx+1]) for pIdx in range(-1, len(robot) - 1)]
        if not self.convex:
            for c in self.child:
                if c.checkCollision(robot):
                    return True
            return False
        else:
            return self._checkCollisionConvex(robot, robotFuncList)

'''
Set up matplotlib to create a plot with an empty square
'''
def setupPlot():
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()
    ax.set_axisbelow(True)
    ax.set_ylim(-1, 11)
    ax.set_xlim(-1, 11)
    ax.grid(which='minor', linestyle=':', alpha=0.2)
    ax.grid(which='major', linestyle=':', alpha=0.5)
    return fig, ax

'''
Make a patch for a single pology 
'''
def createPolygonPatch(polygon, color):
    verts = []
    codes= []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0], xy[1]))
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=1)

    return patch
    

'''
Render the problem  
'''
def drawProblem(robotStart, robotGoal, polygons):
    _, ax = setupPlot()
    patch = createPolygonPatch(robotStart, 'green')
    ax.add_patch(patch)    
    patch = createPolygonPatch(robotGoal, 'red')
    ax.add_patch(patch)    
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p], 'gray')
        ax.add_patch(patch)    
    plt.show()

'''
Grow a simple RRT 
'''
def growSimpleRRT(points):
    newPoints = {}
    adjListMap = {}
    
    # Your code goes here

    T = Tree(points)
    T.grow()
    newPoints, adjListMap = T.getDict()
    
    return newPoints, adjListMap

'''
Perform basic search 
'''
def basicSearch(tree, start, goal):
    path = []
    
    # Your code goes here. As the result, the function should
    # return a list of vertex labels, e.g.
    #
    # path = [23, 15, 9, ..., 37]
    #
    # in which 23 would be the label for the start and 37 the
    # label for the goal.
    bfs = BFS(tree)
    path = bfs.search(start, goal)
    
    return path

'''
Display the RRT and Path
'''
def displayRRTandPath(points, adjListMap, path, robotStart=None, robotGoal=None, polygons=None):
    # Your code goes here
    # You could start by copying code from the function
    # drawProblem and modify it to do what you need.
    # You should draw the problem when applicable.

    _, ax = setupPlot()
    if robotStart is not None:
        patch = createPolygonPatch(robotStart, 'green')
        ax.add_patch(patch)
    if robotGoal is not None:
        patch = createPolygonPatch(robotGoal, 'red')
        ax.add_patch(patch)
    if polygons is not None:
        for p in range(0, len(polygons)):
            patch = createPolygonPatch(polygons[p], 'gray')
            ax.add_patch(patch)

    #vertices
    for vx, vy in points.values():
        ax.plot(vx, vy, 'b.-')
    #edges
    for uIdx, vIdxList in adjListMap.items():
        for vIdx in vIdxList:
            if uIdx < vIdx:
                continue
            ux, uy = points[uIdx]
            vx, vy = points[vIdx]
            ax.plot([ux, vx], [uy, vy], 'k,-')
    #path
    pathSize = len(path)
    if pathSize >= 2:
        for i in range(pathSize - 1):
            ux, uy = points[path[i]]
            vx, vy = points[path[i+1]]
            ax.plot([ux, vx], [uy, vy], c = 'orange')

    plt.show()
        
    return

'''
Collision checking
'''
def isCollisionFree(robot, point, obstacles):

    # Your code goes here.
    robotPoly = [(x + point[0], y + point[1]) for x, y in robot]
    for x, y in robotPoly:
        if x < 0 or x > 10 or y < 0 or y > 10:
            return False
    for polygon in obstacles:
        obs = obstacle(polygon)
        if obs.checkCollision(robotPoly):
            return False
    return True

'''
The full RRT algorithm
'''
def RRT(robot, obstacles, startPoint, goalPoint):

    points = dict()
    tree = dict()
    path = []
    # Your code goes here.
    sampleSize = 500
    maxSmapleSize = 16000
    sIdx = sampleSize + 1
    gIdx = sampleSize + 2
    while True:
        samples = 10 * np.random.rand(sampleSize, 2)
        points = {i+1: tuple(samples[i]) for i in range(sampleSize)}
        points[1] = (5, 5)
        points[sIdx] = startPoint
        points[gIdx] = goalPoint
        T = Tree(points)
        T.grow(obstacles = obstacles, robot = robot)
        points, tree = T.getDict()
        # if sIdx not in tree or gIdx not in tree:
        #     sampleSize = sampleSize * 2
        #     sIdx = sampleSize + 1
        #     gIdx = sampleSize + 2
        tempSIdx = None
        tempGIdx = None
        treeSize = len(points)
        for idx in range(treeSize - 3, treeSize + 1):
            if points[idx] == startPoint:
                tempSIdx = idx
            if points[idx] == goalPoint:
                tempGIdx = idx
        if tempSIdx is not None and tempGIdx is not None:
            #find the path, done
            break
        if sampleSize > maxSmapleSize:
            #too hard
            return points, tree, path

        sampleSize = sampleSize * 2
        sIdx = sampleSize + 1
        gIdx = sampleSize + 2

    path = basicSearch(tree, tempSIdx, tempGIdx)

    
    return points, tree, path

def main(filename, x1, y1, x2, y2, display=''):
    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    robot = []
    obstacles = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            xy = xys[p].split(',')
            polygon.append((float(xy[0]), float(xy[1])))
        if line == 0 :
            robot = polygon
        else:
            obstacles.append(polygon)

    # Print out the data
    print("Robot:")
    print(str(robot))
    print("Pologonal obstacles:")
    for p in range(0, len(obstacles)):
        print(str(obstacles[p]))
    print("")

    # Visualize
    if display == 'display':
        robotStart = [(x + x1, y + y1) for x, y in robot]
        robotGoal = [(x + x2, y + y2) for x, y in robot]
        drawProblem(robotStart, robotGoal, obstacles)

    # Example points for calling growSimpleRRT
    # You should expect many mroe points, e.g., 200-500
    points = dict()
    points[1] = (5, 5)
    points[2] = (7, 8.2)
    points[3] = (6.5, 5.2)
    points[4] = (0.3, 4)
    points[5] = (6, 3.7)
    points[6] = (9.7, 6.4)
    points[7] = (4.4, 2.8)
    points[8] = (9.1, 3.1)
    points[9] = (8.1, 6.5)
    points[10] = (0.7, 5.4)
    points[11] = (5.1, 3.9)
    points[12] = (2, 6)
    points[13] = (0.5, 6.7)
    points[14] = (8.3, 2.1)
    points[15] = (7.7, 6.3)
    points[16] = (7.9, 5)
    points[17] = (4.8, 6.1)
    points[18] = (3.2, 9.3)
    points[19] = (7.3, 5.8)
    points[20] = (9, 0.6)

    # Printing the points
    print("")
    print("The input points are:")
    print(str(points))
    print("")
    
    points, adjListMap = growSimpleRRT(points)
    print("")
    print("The new points are:")
    print(str(points))
    print("")
    print("")
    print("The tree is:")
    print(str(adjListMap))
    print("")

    # Search for a solution  
    # change 1 and 20 as you want
    path = basicSearch(adjListMap, 1, 2)
    print("")
    print("The path is:")
    print(str(path))
    print("")

    # Your visualization code 
    if display == 'display':
        displayRRTandPath(points, adjListMap, path) 

    # Solve a real RRT problem
    points, adjListMap, path = RRT(robot, obstacles, (x1, y1), (x2, y2))
    
    # Your visualization code 
    if display == 'display':
        displayRRTandPath(points, adjListMap, path, robotStart, robotGoal, obstacles) 


if __name__ == "__main__":
    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print("Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]")
        exit()
    
    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])
    display = ''
    if(len(sys.argv) == 7):
        display = sys.argv[6]

    main(filename, x1, y1, x2, y2, display)