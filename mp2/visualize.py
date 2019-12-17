import sys

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np

# ======= the following lines are added to draw roadmap and path =======
import spr

def drawMap(vertexMap, adjListMap, x1, y1, x2, y2, start, goal, fig, ax):
    s = [x1, y1]
    g = [x2, y2]
    vertexMap[start] = s
    vertexMap[goal] = g
    for p, values in adjListMap.items():
        for q, d in values:
            if p < q:
                px, py = vertexMap[p]
                qx, qy = vertexMap[q]
                ax.plot([px, qx], [py, qy], 'g')
    return

def drawPath(vertexMap, path, x1, y1, x2, y2, start, goal, fig, ax):
    s = [x1, y1]
    g = [x2, y2]
    vertexMap[start] = s
    vertexMap[goal] = g
    hopNum = len(path) - 1
    for i in range(hopNum):
        px, py = vertexMap[path[i]]
        qx, qy = vertexMap[path[i + 1]]
        ax.plot([px, qx], [py, qy], 'r')
    return

# ========= the above lines are added to draw roadmap and path =========

'''
Set up matplotlib to create a plot with an empty square
'''
def setupPlot():
    fig = plt.figure(num=None, dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()
    ax.set_axisbelow(True)
    ax.grid(which='minor', linestyle=':', alpha=0.2)
    ax.grid(which='major', linestyle=':', alpha=0.5)
    return fig, ax

'''
Make a patch for a single pology 
'''
def createPolygonPatch(polygon):
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
    patch = patches.PathPatch(path, facecolor='gray', lw=1)

    return patch
    
'''
Make a patch for the robot
'''
def createPolygonPatchForRobot(polygon):
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
    patch = patches.PathPatch(path, facecolor='gray', lw=1)

    return patch
    

'''
Render polygon obstacles  
'''
def drawPolygons(polygons, fig, ax):
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p])
        ax.add_patch(patch)    



if __name__ == "__main__":
    
    # Retrive file name for input data
    if(len(sys.argv) < 2):
        print("Please provide input tfile: python visualize.py [env-file]")
        exit()
    
    # ===== the following lines are added to process start and goal =====
    elif 2 < len(sys.argv) < 6:
        print("Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]")
        exit()
    elif len(sys.argv) >= 6:
        fullFlag = True
        x1 = float(sys.argv[2])
        y1 = float(sys.argv[3])
        x2 = float(sys.argv[4])
        y2 = float(sys.argv[5])
    # ======= the above lines are added to process start and goal =======


    filename = sys.argv[1]

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

    # Setup
    fig, ax = setupPlot()

    # Draw the polygons
    drawPolygons(polygons, fig, ax)

    # Extra visualization elements goes here
    # ===== the following lines are added to draw roadmap and path =====
    if len(sys.argv) >= 6:
        reflexVertices = spr.findReflexiveVertices(polygons)
        vertexMap, adjListMap = spr.computeSPRoadmap(polygons, reflexVertices)
        start, goal, updatedALMap = spr.updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2)
        path, length = spr.uniformCostSearch(updatedALMap, start, goal)
        drawMap(vertexMap, updatedALMap, x1, y1, x2, y2, start, goal, fig, ax)
        drawPath(vertexMap, path, x1, y1, x2, y2, start, goal, fig, ax)
    # ======= the above lines are added to draw roadmap and path =======
    
    plt.show()