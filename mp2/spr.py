import sys
import numpy as np
from collections import defaultdict

import itertools as it
import queue

def getDist(p, q):
    px, py = p
    qx, qy = q
    return np.sqrt(np.square(px - qx) + np.square(py - qy))

def getDet(p, v, n):
    px, py = p
    vx, vy = v
    nx, ny = n
    return (vx - nx) * (py - vy) - (px - vx) * (vy - ny)

def checkBitan(p, q, pList, qList):
    if len(pList) != 2:
        # print('E: pList')
        # print(pList)
        # print(p)
        # print(q)
        return False
    if len(qList) != 2:
        # print('E: qList')
        # print(qList)
        # print(q)
        # print(p)
        return False

    a, b = pList
    c, d = qList
    if getDet(q, p, a) * getDet(q, p, b) < 0:
        return False
    if getDet(p, q, c) * getDet(p, q, d) < 0:
        return False

    return True

def generateLineFunc(p, q):
    px, py = p
    qx, qy = q
    def lineFunc(point):
        x, y = point
        return (x - px) * (qy - py) - (y - py) * (qx - px)
    return lineFunc

def getLineFuncList(polygon):
    lineFuncList = []
    lineSegmentP = []

    pointNum = len(polygon)
    if pointNum == 0 or pointNum == 1:
        return lineFuncList

    for i in range(pointNum):
        j = (i - 1) % pointNum #actually there is no need to use % pointNum
        p = polygon[i]
        q = polygon[j]
        lineFuncList.append(generateLineFunc(p, q))
        lineSegmentP.append([p, q])
    return lineFuncList, lineSegmentP

def getPolygonLineFuncList(polygons):
    polyLineFuncList = []
    polyLineSegmentP = []
    if len(polygons) == 0:
        return polyLineFuncList

    for polygon in polygons:
        lineFuncList, lineSegmentP = getLineFuncList(polygon)
        polyLineFuncList.extend(lineFuncList)
        polyLineSegmentP.extend(lineSegmentP)
    return polyLineFuncList, polyLineSegmentP

def checkIntersect(p, q, polyLineFuncList, polyLineSegmentP, checkBitangent = False):
    tempLineFunc = generateLineFunc(p, q)
    lineNum = len(polyLineFuncList)
    pList = []
    qList = []

    if lineNum == 0:
        return False
    for i in range(lineNum):
        polyLineFunc = polyLineFuncList[i]
        a, b = polyLineSegmentP[i]
        if (p == a and q == b) or (p == b and q == a):
            #if they are consecutive reflex vertices, done
            return False

        if polyLineFunc(p) * polyLineFunc(q) < 0 and tempLineFunc(a) * tempLineFunc(b) < 0:
            return True

        if p == a and q != b:
            pList.append(b)
        elif p == b and q != a:
            pList.append(a)
        elif p != a and q == b:
            qList.append(a)
        elif p != b and q == a:
            qList.append(b)

    if checkBitangent:
        if not checkBitan(p, q, pList, qList):
            return True

    return False

# ======= the following lines are added after grading to fix bugs =======
def checkInside(p, q, polygons):
    tempLineFunc = generateLineFunc(p, q)
    for polygon in polygons:
        if len(polygon) <= 3:
            #no way to be inside, done
            continue

        count = 0
        posFlag = False
        negFlag = False
        for v in polygon:
            if v == p or v == q:
                count = count + 1
            elif tempLineFunc(v) > 0:
                posFlag = True
            elif tempLineFunc(v) < 0:
                negFlag = True
            if posFlag and negFlag and count == 2:
                return True
    return False
# ========= the above lines are added to after grading to fix bugs =========

def getPath(prevDict, start, goal):
    path = []
    tempV = goal
    while tempV != start:
        path.append(tempV)
        tempV = prevDict[tempV]
    path.append(start)
    path.reverse()
    return path

'''
Report reflexive vertices
'''
def findReflexiveVertices(polygons):
    vertices=[]
    
    # Your code goes here
    # You should return a list of (x,y) values as lists, i.e.
    # vertices = [[x1,y1],[x2,y2],...]

    if len(polygons) == 0:
        #no polygons, done
        return vertices

    for polygon in polygons:
        pointNum = len(polygon)
        if pointNum == 0:
            #no points, done
            continue
        elif pointNum == 1 or pointNum == 2:
            #single point or straight line, add by def.
            for point in polygon:
                vertices.append(point)
            continue

        for vIdx in range(pointNum):
            pIdx = (vIdx - 1) % pointNum
            nIdx = (vIdx + 1) % pointNum
            #p->v->n
            px, py = polygon[pIdx]
            vx, vy = polygon[vIdx]
            nx, ny = polygon[nIdx]

            if vx > 10 or vx < 0 or vy > 10 or vy < 0:
                #outside, done
                continue

            if getDet(polygon[pIdx], polygon[vIdx], polygon[nIdx]) > 0:
                #det(v - n, p - v) > 0 => "right" turn
                vertices.append(polygon[vIdx])
    
    return vertices

'''
Compute the roadmap graph
'''
def computeSPRoadmap(polygons, reflexVertices):
    vertexMap = dict()
    adjacencyListMap = defaultdict(list)
    
    # Your code goes here
    # You should check for each pair of vertices whether the
    # edge between them should belong to the shortest path
    # roadmap. 
    #
    # Your vertexMap should look like
    # {1: [5.2,6.7], 2: [9.2,2.3], ... }
    #
    # and your adjacencyListMap should look like
    # {1: [[2, 5.95], [3, 4.72]], 2: [[1, 5.95], [5,3.52]], ... }
    #
    # The vertex labels used here should start from 1
    
    reflexNum = len(reflexVertices)
    if reflexNum == 0:
        #no reflexV, done
        return vertexMap, adjacencyListMap
    if reflexNum == 1:
        #a single reflexV, add to map, done
        vertexMap[1] = reflexVertices[0]
        return vertexMap, adjacencyListMap

    #compute vertexMap
    for i, v in enumerate(reflexVertices):
        vertexMap[i + 1] = v

    #compute adjacencyListMap
    polyLineFuncList, polyLineSegmentP = getPolygonLineFuncList(polygons)

    for i, j in it.combinations(range(1, reflexNum + 1), 2):
        p = vertexMap[i]
        q = vertexMap[j]
        if checkIntersect(p, q, polyLineFuncList, polyLineSegmentP, checkBitangent = True):
            continue
        dist = getDist(p, q)
        adjacencyListMap[i].append([j, dist])
        adjacencyListMap[j].append([i, dist])

    return vertexMap, adjacencyListMap

'''
Perform uniform cost search 
'''
def uniformCostSearch(adjListMap, start, goal):
    path = []
    pathLength = 0
    
    # Your code goes here. As the result, the function should
    # return a list of vertex labels, e.g.
    #
    # path = [23, 15, 9, ..., 37]
    #
    # in which 23 would be the label for the start and 37 the
    # label for the goal.
    fringe = queue.PriorityQueue()
    closed = set()
    prevDict = {start: start}
    distDict = dict()


    fringe.put((0, start))
    while not fringe.empty():
        tempD, tempV = fringe.get()
        if tempV in closed:
            continue
        closed.add(tempV)

        if tempV == goal:
            path = getPath(prevDict, start, goal)
            pathLength = tempD
            break
        for nextV, dist in adjListMap[tempV]:
            if nextV in closed:
                continue

            nextD = tempD + dist
            if nextV in prevDict:
                if distDict[nextV] < nextD:
                    continue

            distDict[nextV] = nextD
            prevDict[nextV] = tempV
            fringe.put((nextD, nextV))

    return path, pathLength

'''
Agument roadmap to include start and goal
'''
def updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2):
    updatedALMap = dict()
    startLabel = 0
    goalLabel = -1

    # Your code goes here. Note that for convenience, we 
    # let start and goal have vertex labels 0 and -1,
    # respectively. Make sure you use these as your labels
    # for the start and goal vertices in the shortest path
    # roadmap. Note that what you do here is similar to
    # when you construct the roadmap. 

    s = [x1, y1]
    g = [x2, y2]
    reflexNum = len(vertexMap)
    vertexMap[startLabel] = s
    vertexMap[goalLabel] = g

    polyLineFuncList, polyLineSegmentP = getPolygonLineFuncList(polygons)
    if not checkIntersect(s, g, polyLineFuncList, polyLineSegmentP, checkBitangent = False):
        if not checkInside(s, g, polygons):
            dist = getDist(s, g)
            adjListMap[startLabel].append([goalLabel, dist])
            adjListMap[goalLabel].append([startLabel, dist])


    for i in range(1, reflexNum + 1):
        p = vertexMap[i]
        if checkIntersect(s, p, polyLineFuncList, polyLineSegmentP, checkBitangent = False):
            continue
        if checkInside(s, p, polygons):
            continue
        dist = getDist(s, p)
        adjListMap[startLabel].append([i, dist])
        adjListMap[i].append([startLabel, dist])

    for i in range(1, reflexNum + 1):
        p = vertexMap[i]
        if checkIntersect(g, p, polyLineFuncList, polyLineSegmentP, checkBitangent = False):
            continue
        if checkInside(g, p, polygons):
            continue
        dist = getDist(g, p)
        adjListMap[goalLabel].append([i, dist])
        adjListMap[i].append([goalLabel, dist])

    updatedALMap = adjListMap

    return startLabel, goalLabel, updatedALMap


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

    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    polygons = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            polygon.append([float(i) for i in xys[p].split(',')])
        polygons.append(polygon)

    # Print out the data
    print("Pologonal obstacles:")
    for p in range(0, len(polygons)):
        print(str(polygons[p]))
    print("")

    # Compute reflex vertices
    reflexVertices = findReflexiveVertices(polygons)
    print("Reflexive vertices:")
    print(str(reflexVertices))
    print("")

    # Compute the roadmap 
    vertexMap, adjListMap = computeSPRoadmap(polygons, reflexVertices)
    print("Vertex map:")
    print(str(vertexMap))
    print("")
    print("Base roadmap:")
    print(dict(adjListMap))
    print("")

    # Update roadmap
    start, goal, updatedALMap = updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2)
    print("Updated roadmap:")
    print(dict(updatedALMap))
    print("")

    # Search for a solution     
    path, length = uniformCostSearch(updatedALMap, start, goal)
    print("Final path:")
    print(str(path))
    print("Final path length:" + str(length))
