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

treeHeights = [-1, -1, -1, -1, -1, -1, -1, -1, -1]
treeRotations = [0, 0, 0, 0, 0, 0, 0, 0, 0]
# 0 = Not harvested, 1 = Harvested, 2 = Detection Failed, 3 = Harvest Failed
fruitStatus = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

def GetTreePosition(treeNum):
    return { treeLocations[2*treeNum], treeLocations[2*treeNum + 1] }

def DetermineNextFruit(currentTree):
    
    pass