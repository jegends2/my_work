import fields2cover as f2c
from osgeo import ogr




def main():

  # 1. initialize a F2CPoint
  # 1) using X and Y coordinates
  p1 = f2c.Point(1.2, 3.4)
  print("Point 1: ", p1)

  # 2) using X, Y, Z coordinates
  #p2 = f2c.Point(9.8, 7.6, 5.4)
  #print("Point 2: ", p2)

  # 3) using OGRPoint from GDAL
  #ogrpoint = ogr.Geometry(ogr.wkbPoint)
  #ogrpoint.AddPoint(11, 22)
  #p3 = f2c.Point()
  #p3.importFromWkt(ogrpoint.ExportToWkt())
  #print("Point 3: ", p3)

  # 4) creating an empty F2CPoint and setting its commponents using setX / setY / setZ the components can be also read with getX / getY / getZ
  p4 = f2c.Point()
  p4.setX(3.0)
  p4.setZ(0.0)
  print("Point 4: ", p4,
      ". Its components are: {x: ", p4.getX(),
      ", y: ", p4.getY(),
      ", z: ", p4.getZ(), "}")

  # 5) creating an empty F2CPoint and importing its components
  p5 = f2c.Point()
  p5.importFromWkt("POINT (0 4 0)")
  print("Point 5: ", p5)

  # 2. Basic types are shared pointers
  # classes derived from GDAL types, like F2CPoint from OGRPoint, use a compound structure of shared pointers on it.
  # The pointers to GDAL types can be access as p1->() or p1.get(). Usually, this is not needed, as a simple access has been provided
  distance = p4.distance(p5)
  print("Access to OGRPoints: ", distance)
  print("Without accessing: ", p4.distance(p5))

  # 3. initialize a F2CLineString
  # a F2CLineString (f2c::types::LineString) is a line defined by a vector of points. To initialize a F2CLineString, we can
  # 1) create an empty F2CLineString and adding several F2CPoint
  line1 = f2c.LineString()
  line1.addPoint(3,0)
  line1.addPoint(p5)
  print("Length of line 1: ", line1.length())

  # 2) give a sequence of F2CPoint
  line2 = f2c.LineString()
  [line2.addPoint(p) for p in [f2c.Point(1, 0), f2c.Point(1, 1), f2c.Point(0, 1)]]
  print("Length of line 2: ", line2.length())

  # 4. Initialize a F2CLinearRing
  # a F2CLinearRing (f2c::types::LinearRing) is a closed F2CLineString. It can be initialized as a F2CLineString
  ring = f2c.LinearRing()
  [ring.addPoint(p) for p in [f2c.Point(1,1), f2c.Point(1,2), f2c.Point(2,2), f2c.Point(1,1)]]
  print("Area of the ring: ", ring.area())
  # the main difference between F2CLineString and F2CLinearRing is that F2CLinearRing is expected to be closed, so the area can be computed.

  # 5. Initializing other collections
  # a F2CMultiLineString (f2c::types::MultiLineString) are several F2CLineString. It can be initialize as
  lines = f2c.MultiLineString()
  lines.addGeometry(line1)
  lines.addGeometry(line2)
  print("Lines have length: ", end="")
  for i in range(lines.size()):
    print(lines.getGeometry(i).length(), end = ", ")
  print("\n")

  # a F2CCell (f2c::types::Cell) is a polygon created by one outter F2CLinearRing and zero, one or many inner F2CLinearRing. The first F2CLinearRing is the outter one. Moreover, all the F2CLinearRing should not intersect with each others.
  outter_ring = f2c.LinearRing()
  [outter_ring.addGeometry(p) for p in [  \
    f2c.Point(0, 0), f2c.Point(2, 0), f2c.Point(2, 2), f2c.Point(0, 2), f2c.Point(0, 0)]]
  inner_ring = f2c.LinearRing()
  [inner_ring.addGeometry(p) for p in [  \
    f2c.Point(0.5, 0.5), f2c.Point(1.5, 0.5), f2c.Point(1.5, 1.5),  \
    f2c.Point(0.5, 1.5), f2c.Point(0.5, 0.5)]]
  cell = f2c.Cell()
  cell.addRing(outter_ring)
  cell.addRing(inner_ring)
  print("The area of the cell is: ", cell.area(), "\n")

  # a F2CCells (f2c::types::Cells) is a multipolygon. It contains zero, one or several F2CCell on it.
  cells = f2c.Cells()
  cells.addGeometry(cell)
  print("The area of the cells is: ", cells.area(), "\n\n")

  # lastly, F2CMultiPoint (f2c::types::MultiPoint) is a collection of F2CPoint
  points = f2c.MultiPoint()
  [points.addGeometry(p) for p in [f2c.Point(1, 2), f2c.Point(3, 4)]]
  print("Points contains ", points.size(), " points.")
  points.addPoint(5, 6)
  print("Points contains ", points.size(), " points.")
  points.addPoint(p5)
  print("Points contains ", points.size(), " points.")

  # 6. Accessing elements in collections
  # to access each of the elements in a collection, the function getGeometry(int n) returns the element n.
  p_0 = points.getGeometry(0)
  print("First point in points: ", p_0, "\n")

  # unfortunately, if we change the child element, it is not changed on the collection. If we want to keep it, we have to set the geometry back with setGeometry()
  p_0 *= 1e5
  print("Modified p_0: ", p_0)
  print("First point in points without modification: ", points.getGeometry(0))
  points.setGeometry(0, p_0)
  print("Modified first point in points: ", points.getGeometry(0))
  # this process can be done in any of the collection types presented previously: F2CLineString, F2CLinearRing, F2CMultiLineString, F2CCell, F2CCells and F2CMultiPoint

  # 7. F2CRobot
  # the vehicle to cover the field is defined as a F2CRobot struct. To initialize it, the constructor needs the width of the robot and the width of the operation. For example, if we have a vehicle to fertilize a field, with 3m width and a 39m operational width, we should initialize it as
  robot = f2c.Robot(3.0, 39.0)
  # robot import functions (F2CRobot)
  # getWidth/setWidth = if something is closer than this value from the robot, we can expect it will be hit
  # getCovWidth/setCovWidth: get/set the coverage width of the robot, also called operational width. This parameter defines the width of the swaths in the field
  # getMinTurningRadius/setMinTurningRadius and getMaxCurv/setMaxCurv: get/set the minimim turning radius or the maximum, respectively. Both are saved as the same parameter, as maximim curvature is the invers of the minimum turning radius
  # getMaxDiffCurv/setCruiseVel: get/set the speed of the vehicle when traveling through the field
  # getTurnVel/setTurnVel: get/set the speed of the vehicle when making turns or going through the headlands

  # F2CSwath / F2CSwaths / F2CSwathsByCells
  # F2CRoute
  # F2CPath
  f2c.Visualizer.figure()
  f2c.Visualizer.plot(lines)
  f2c.Visualizer.show()
#  f2c.Visualizer.save("test_image.pgm")

  print("Lines geometry: ", lines.getGeometry(0))
  print("Line length: ", lines.getGeometry(0).length())

if __name__ == "__main__":
    main()