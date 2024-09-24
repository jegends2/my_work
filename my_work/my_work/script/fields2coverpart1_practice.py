import fields2cover as f2c

def main():
    p1 = f2c.Point(1, 0)
    print("Point 1: ", p1)
    p5 = f2c.Point(0, 5)
    print("Point 5 ", p5)

    line1 = f2c.LineString()
    line1.addPoint(3,0)
    line1.addPoint(p5)
    print("Length of line1: ", line1.length())

    line2 = f2c.LineString()
    [line2.addPoint(p) for p in [f2c.Point(1, 0), f2c.Point(1, 1), f2c.Point(0,3)]]
    print("Length of line 2: ", line2.length())

    ring = f2c.LinearRing()
    [ring.addPoint(p) for p  in [f2c.Point(1,1), f2c.Point(1,2), f2c.Point(0.5,4), f2c.Point(0,3)]]
    print("Area of the ring: ", ring.area())

    lines = f2c.MultiLineString()
    lines.addGeometry(line1)
    lines.addGeometry(line2)
    print("Lines have length: ", end="")
    for i in range(lines.size()):
        print(lines.getGeometry(i).length(), end = ", ")
    print("\n")
    
    outer_ring = f2c.LinearRing()
    [outer_ring.addGeometry(p) for p in [  \
        f2c.Point(0,0), f2c.Point(2,0), f2c.Point(2,2), f2c.Point(0,2), f2c.Point(0,0)]]
    inner_ring = f2c.LinearRing()
    [inner_ring.addGeometry(p) for p in [  \
        f2c.Point(0.5,0.5), f2c.Point(1.5,0.5), f2c.Point(1.5,1.5),  \
        f2c.Point(0.5,1.5), f2c.Point(0.5,0.5)]]
    cell = f2c.Cell()
    cell.addRing(outer_ring)
    cell.addRing(inner_ring)

    f2c.Visualizer.figure()
    f2c.Visualizer.plot(lines)
    f2c.Visualizer.plot(cell)
    f2c.Visualizer.show()


if __name__ == "__main__":
    main()