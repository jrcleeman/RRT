class Env:
    def __init__(self, xMin = 0, xMax = 50, yMin = 0, yMax = 30):
        #Enviorment X Min and Max
        self.xRange = (xMin, xMax)
        #Enviornment Y Min and Max
        self.yRange = (yMin, yMax)
        #Create boundary and obstacles
        self.buildBoundary()
        self.buildCircleObs()
        self.buildRectangleObs()

    def buildBoundary(self):
        #(x, y, w(x), h(y))
        self.boundary = [
            [0, 0, 1, 30],
            [0, 30, 50, 1],
            [1, 0, 50, 1],
            [50, 1, 1, 30]
        ]
        return None
    
    def buildCircleObs(self):
        #(x, y, radius)
        self.circleObstacles = [
            [40, 19, 2]
        ]
        return None    

    def buildRectangleObs(self):
        #(x, y, w(x), h(y))
        self.rectangleObstacles = [
            [25, 1, 2, 11],
            [25, 16, 2, 14]
        ]
        return None