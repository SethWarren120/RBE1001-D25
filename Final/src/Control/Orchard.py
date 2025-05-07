treeLocations = [
    1, 2,
    1, 2,
    1, 2,
    1, 2,
    1, 2,
    1, 2,
    1, 2,
    1, 2,
    1, 2
]

treeHeightIntervals = [0, 3, 6, 9, 12, 15, 18, 21]
treeHeights = [-1, -1, -1, -1, -1, -1, -1, -1, -1]
treeHeightSums = [-1, -1, -1, -1, -1, -1, -1, -1, -1]
treeHeightCounts = [-1, -1, -1, -1, -1, -1, -1, -1, -1]
treeRotations = [0, 0, 0, 0, 0, 0, 0, 0, 0]
treeRotationCertainty = [-1, -1, -1, -1, -1, -1, -1, -1, -1]
# 0 = Not harvested, 1 = Harvested, 2 = Detection Failed, 3 = Harvest Failed
fruitStatus = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

def GetTreePosition(treeNum):
    return { treeLocations[2*treeNum], treeLocations[2*treeNum + 1] }

def DetermineNextFruit(currentTree):
    
    pass

def GiveTreeHeightEstimate(tree, height, certainty):
    treeHeightSums[tree] += height
    treeHeightCounts[tree] += certainty

    bestHeight = 0
    i = 1
    while i < 8:
        if abs((treeHeightSums[tree] / treeHeightCounts[tree]) - treeHeightIntervals[i]) < abs((treeHeightSums[tree] / treeHeightCounts[tree]) - treeHeightIntervals[bestHeight]):
            bestHeight = i
    
    treeHeights[tree] = bestHeight

def GiveTreeRotationEstimate(tree, rotation, certainty):
    if certainty >= treeRotationCertainty[tree]:
        treeRotations[tree] = rotation
        treeRotationCertainty[tree] = certainty