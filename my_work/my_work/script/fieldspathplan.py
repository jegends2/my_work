import math
import fields2cover as f2c

def read_polygon_from_file(file_path):
    """Read polygon points from a file."""
    with open(file_path, "r") as f:
        lines = f.readlines()
    return [(float(line.split(",")[0]), float(line.split(",")[1])) for line in lines]

def create_field_from_polygon(polygon):
    """Create a field from polygon points."""
    # Create outer ring
    outer_ring = f2c.LinearRing()
    for x, y in polygon:
        outer_ring.addPoint(f2c.Point(x, y))
    
    # Create cell and add outer ring
    cell = f2c.Cell()
    cell.addRing(outer_ring)

    print("The area of the cell is: ", cell.area())

    # Create a Field object and add the cell to it
    field = f2c.Field()
    # Assuming that Field has a method to add cells, but if not, use the available methods
    # Check documentation or use dir() to list available methods
    # field.addCell(cell)
    # If no addCell, see if there are other methods to create or initialize the field
    
    return field

def main():
    # Load polygon points
    polygon = read_polygon_from_file("polygon.txt")

    # Create field from polygon
    try:
        field = create_field_from_polygon(polygon)
    except AttributeError as e:
        print(f"Error in creating field: {e}")
        return
    except TypeError as e:
        print(f"TypeError in creating field: {e}")
        return

    # Define robot
    robot = f2c.Robot(2.0, 6.0)
    # Use a method to set minimum turning radius if needed
    robot.setMinTurningRadius(2)

    # Generate headlands and swaths
    const_hl = f2c.HG_Const_gen()
    no_hl = const_hl.generateHeadlands(field, 3.0 * robot.getWidth())
    bf = f2c.SG_BruteForce()
    swaths = bf.generateSwaths(math.pi, robot.getCovWidth(), no_hl.getGeometry(0))

    # Sort swaths
    snake_sorter = f2c.RP_Snake()
    swaths = snake_sorter.genSortedSwaths(swaths)

    # Plan path
    path_planner = f2c.PP_PathPlanning()
    dubins = f2c.PP_DubinsCurves()
    path = path_planner.planPath(robot, swaths, dubins)

    # Save path
    path_gps = f2c.Transform.transformToPrevCRS(path, field)
    path_gps.saveToFile("path.csv")

    print("Path has been planned and saved to path.csv")

if __name__ == "__main__":
    main()
