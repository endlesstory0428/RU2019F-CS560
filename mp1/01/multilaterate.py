import sys
import numpy as np
import itertools as it

def getDist(c1, c2):
    #return ||c2 - c1||, normalize(c2 - c1)
    vect = c2 - c1
    dist = np.sqrt(np.sum(np.square(vect)))
    if dist == 0:
        print('E: too close')
        exit()
    return dist, (vect / dist)


def chkDist(cList, rList):
    #return distList[i, j], normList[i, j] = getDist(i, j)
    distThreshold = 1e-6 #TODO: check

    distList = np.zeros((4, 4), dtype = np.float64)
    normList = np.zeros((4, 4, 3), dtype = np.float64)
    for i in range(4):
        for j in range(4):
            if i != j:
                dist, vect = getDist(cList[i], cList[j])
                if dist < distThreshold * np.mean(rList[[i, j]]): #DO NOT delete double []
                    print('E: too close')
                    exit()
                if dist > (1 + distThreshold) * (rList[i] + rList[j]):
                    print('E: too far away')
                    exit()
                distList[i, j] = dist
                normList[i, j] = vect
            elif i == j:
                continue
            else:
                distList[i, j] = distList[j, i]
                normList[i, j] = -normList[j, i]
    return distList, normList


def getMaxDist3Comb(distList):
    #return index i that maximize the minimum dist to the others
    combDistList = np.zeros((4, 3), dtype = np.float64)
    idxSet = set(range(4))
    for i in idxSet:
        leftIdxList = idxSet - {i}
        for jIdx, j in enumerate(leftIdxList):
            combDistList[i, jIdx] = distList[i, j]
    maxIdx = np.argmax(np.min(combDistList, axis = 1))
    return maxIdx, list(idxSet - {maxIdx})


def chkFulRank(distList, normList):
    #check [c1 - c2, c2 - c3, c3 - c4] is full ranked
    rankThreshold = 1e-6 #TODO: check
    fulRankFlag = False

    maxIdx, leftIdxList = getMaxDist3Comb(distList)
    planeNormVList = np.empty((3, 3), dtype = np.float64)
    for jIdx, j in enumerate(leftIdxList):
        planeNormVList[jIdx] = normList[maxIdx, j]
    if np.abs(np.linalg.det(planeNormVList)) >= rankThreshold:
        fulRankFlag = True

    return fulRankFlag, maxIdx, leftIdxList


def getIntersectC(s1, s2, dist, normV):
    #get the intersection circle center
    c1 = s1[0:3]
    r1 = s1[3]
    c2 = s2[0:3]
    r2 = s2[3]
    x = (np.square(dist) - np.square(r2) + np.square(r1)) / (2 * dist)
    return c1 + x * normV #normV = (c2 - c1) / dist


def solveFulRank(distances, distList, normList, maxIdx, leftIdxList):
    #use 3 intersection circle plane to solve location
    planeNormVList = np.empty((3, 3), dtype = np.float64)
    planePointList = np.empty((3, 3), dtype = np.float64)

    for jIdx, j in enumerate(leftIdxList):
        planeNormVList[jIdx] = normList[maxIdx, j]
        planePointList[jIdx] = getIntersectC(distances[maxIdx], distances[j], distList[maxIdx, j], normList[maxIdx, j])
    b = np.sum(planeNormVList * planePointList, axis = 1)

    return np.dot(np.linalg.inv(planeNormVList), b)


def chkLine(distances, distList, i, j, k):
    #check i j k are on the same line
    lineThreshold = 1e-6 * np.min(distances[:, 3][[i, j, k]]) #TODO: check

    tempDistList = [distList[i, j], distList[j, k], distList[i, k]]
    tempDistList.sort()
    if lineThreshold > np.abs(tempDistList[0] + tempDistList[1] - tempDistList[2]):
        return True
    else:
        return False


def chkTwoRank(distances, distList, normList, maxIdx, leftIdxList):
    #check [c1 - c2, c2 - c3, c3 - c4] is 2 ranked
    for jIdx, j in enumerate(leftIdxList):
        for kIdx, k in enumerate(leftIdxList):
            if j <= k:
                continue
            else:
                if chkLine(distances, distList, maxIdx, j, k):
                    continue
                else:
                    return True, (maxIdx, j, k), list(set(leftIdxList) - {j, k})
    return False, (-1, -1, -1), leftIdxList


def solveTwoRank(distances, distList, normList, indexSet):
    #use 2 intersection circle plane and the common plane of 4 centers to solve location
    i, j, k = indexSet
    planeNormVList = np.empty((3, 3), dtype = np.float64)
    planePointList = np.empty((3, 3), dtype = np.float64)

    planeNorm = np.cross(normList[i, j], normList[i, k])
    planeNormVList[2] = planeNorm / np.sqrt(np.sum(np.square(planeNorm)))
    planePointList[2] = distances[i, 0:3]

    planeNormVList[0] = normList[i, j]
    planePointList[0] = getIntersectC(distances[i], distances[j], distList[i, j], normList[i, j])

    planeNormVList[1] = normList[i, k]
    planePointList[1] = getIntersectC(distances[i], distances[k], distList[i, k], normList[i, k])

    b = np.sum(planeNormVList * planePointList, axis = 1)
    print([i, j, k])
    print(planeNormVList)
    print(planePointList)
    print(b)

    return np.dot(np.linalg.inv(planeNormVList), b)


def solveOneRank(distances, normList, i, j):
    #use larger circle r to sovle the tangent point
    r1 = distances[i, 3]
    r2 = distances[j, 3]
    if r1 >= r2:
        location = distances[i, 0:3] + r1 * normList[i, j] #c1 + r1 * (c2 - c1)
    else:
        location = distances[j, 0:3] + r2 * normList[j, i] #c2 + r2 * (c1 - c2)
    return location


def chkLoc(distances, location):
    #check the result
    locThreshold = 1e-6 #TODO: check

    for i in range(4):
        dist, norm = getDist(distances[i, 0:3], location)
        if dist * (1 - locThreshold) <= distances[i, 3] <= dist * (1 + locThreshold):
            continue
        else:
            return False
    return True


def multilaterate(distances):
    distances = np.array(distances)

    distList, normList = chkDist(distances[:, 0:3], distances[:, 3])

    fulRankFlag, maxIdx, leftIdxList = chkFulRank(distList, normList)
    if fulRankFlag:
        location = solveFulRank(distances, distList, normList, maxIdx, leftIdxList)
    else:
        #low ranked
        twoRankFlag, indexSet, leftIdxList = chkTwoRank(distances, distList, normList, maxIdx, leftIdxList)
        if twoRankFlag:
            location = solveTwoRank(distances, distList, normList, indexSet)
        else:
            #4 centers are on the same line
            location = solveOneRank(distances, normList, maxIdx, leftIdxList[0])
    if not chkLoc(distances, location):
        print('E: Not exactly 1 intersection')
        exit()
    return location


if __name__ == "__main__":
    
    # Retrive file name for input data
    if(len(sys.argv) == 1):
        print("Please enter data file name.")
        exit()
    
    filename = sys.argv[1]

    # Read data
    lines = [line.rstrip('\n') for line in open(filename)]
    distances = []
    for line in range(0, len(lines)):
        distances.append(list(map(float, lines[line].split(' '))))

    # Print out the data
    print ("The input four points and distances, in the format of [x, y, z, d], are:")
    for p in range(0, len(distances)):
        print (*distances[p]) 

    # Call the function and compute the location 
    location = multilaterate(distances)
    print 
    print ("The location of the point is: " + str(location))
