

class CameraIntrinsic:
    def __init__(self, width, height, fx, fy, ppx, ppy) -> None:
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.ppx = ppx
        self.ppy = ppy

    def __str__(self) -> str:
        return str(self.__dict__)



################################### Util functions from Nvidia DOPE #####################################

def DrawLine(point1, point2, lineColor, lineWidth,draw):
    if not point1 is None and not point2 is None:
        draw.line([point1,point2],fill=lineColor,width=lineWidth)

def DrawDot(point, pointColor, pointRadius, draw):
    if not point is None:
        xy = [point[0]-pointRadius, point[1]-pointRadius, point[0]+pointRadius, point[1]+pointRadius]
        draw.ellipse(xy, fill=pointColor, outline=pointColor)

def DrawCube(points, which_color = 0, color = None, draw = None):
    '''Draw cube with a thick solid line across the front top edge.'''
    lineWidthForDrawing = 2
    lineWidthThick = 8
    lineColor1 = (255, 215, 0)  # yellow-ish
    lineColor2 = (12, 115, 170)  # blue-ish
    lineColor3 = (45, 195, 35)  # green-ish
    if which_color == 3:
        lineColor = lineColor3
    else:
        lineColor = lineColor1

    if not color is None:
        lineColor = color        

    # draw front
    DrawLine(points[0], points[1], lineColor, lineWidthThick, draw)
    DrawLine(points[1], points[2], lineColor, lineWidthForDrawing, draw)
    DrawLine(points[3], points[2], lineColor, lineWidthForDrawing, draw)
    DrawLine(points[3], points[0], lineColor, lineWidthForDrawing, draw)
    
    # draw back
    DrawLine(points[4], points[5], lineColor, lineWidthForDrawing, draw)
    DrawLine(points[6], points[5], lineColor, lineWidthForDrawing, draw)
    DrawLine(points[6], points[7], lineColor, lineWidthForDrawing, draw)
    DrawLine(points[4], points[7], lineColor, lineWidthForDrawing, draw)
    
    # draw sides
    DrawLine(points[0], points[4], lineColor, lineWidthForDrawing, draw)
    DrawLine(points[7], points[3], lineColor, lineWidthForDrawing, draw)
    DrawLine(points[5], points[1], lineColor, lineWidthForDrawing, draw)
    DrawLine(points[2], points[6], lineColor, lineWidthForDrawing, draw)

    # draw dots
    DrawDot(points[0], pointColor=lineColor, pointRadius = 4,draw = draw)
    DrawDot(points[1], pointColor=lineColor, pointRadius = 4,draw = draw)


###########################################################################################################