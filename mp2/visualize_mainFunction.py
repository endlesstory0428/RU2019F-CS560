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