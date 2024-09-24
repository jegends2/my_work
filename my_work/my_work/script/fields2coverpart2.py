import fields2cover as f2c
import math


#Fields2Cover defines different objective functions for each module: Decomp (Decomposition), HG (Headland Generator), SG (Swath Generator), RP (Route Planner), PP(Path Planner)

def main():
    # HG (Headland Generator) objective functions
    # Remaining area
    total_field = f2c.Cells(f2c.Cell(f2c.LinearRing(f2c.VectorPoint([
        f2c.Point(-2,-2), f2c.Point(6,-2), f2c.Point(6,6),
        f2c.Point(-2,6), f2c.Point(-2,-2)]))))
    field = f2c.Cells(f2c.Cell(f2c.LinearRing(f2c.VectorPoint([
        f2c.Point(0,0), f2c.Point(4,0), f2c.Point(4,4),
        f2c.Point(0,4), f2c.Point(0,0)]))))

    rem_area = f2c.OBJ_RemArea()
    print("The remaining area is ",
        rem_area.computeCost(total_field, field), ", and with sign is ",
        rem_area.computeCostWithMinimizingSign(total_field, field))
    
    # SG objective functions
    # Field coverage
    width = 2.0
    swath1 = f2c.Swath(f2c.LineString(f2c.VectorPoint([
        f2c.Point(0.0, 1.0), f2c.Point(4.0, 1.0)])), width)
    swath2 = f2c.Swath(f2c.LineString(f2c.VectorPoint([
        f2c.Point(0.0, 3.0), f2c.Point(4.0, 3.0)])), width)
    swath3 = f2c.Swath(f2c.LineString(f2c.VectorPoint([
        f2c.Point(0.0, 2.0), f2c.Point(4.0, 2.0)])), width)

    swaths1 = f2c.Swaths()
    swaths1.push_back(swath1)
    swaths2 = f2c.Swaths()
    swaths2.push_back(swath2)
    swaths3 = f2c.Swaths()
    [swaths3.push_back(s) for s in [swath1, swath2, swath3]]

    field_cov = f2c.OBJ_FieldCoverage()
    print("The field coverage with swath1 is ",
        field_cov.computeCost(field, swaths1), " and with all of the swaths ",
        field_cov.computeCost(field, swaths3))
    
    print("The field coverage with sign for all of the swaths is ",
        field_cov.computeCostWithMinimizingSign(field, swaths3))
    
    # Number of swaths
    n_swaths = f2c.OBJ_NSwath()

    print("The number of swaths with swath1 is ",
        n_swaths.computeCost(swaths1), " and with all of the swaths ",
        n_swaths.computeCost(field, swaths3))
    
    n_swaths_mod = f2c.OBJ_NSwathModified()

    print("The number of swaths with swath1 is ",
        n_swaths_mod.computeCost(swaths1), " and with all of the swaths ",
        n_swaths_mod.computeCost(field, swaths3))
    

    # Overlap
    overlaps = f2c.OBJ_Overlaps()

    print("The field overlapping with swath1 is ",
        overlaps.computeCost(field, swaths1), " and with all of the swaths ",
        overlaps.computeCost(field, swaths3))
    
    # Swath Length
    swath_length = f2c.OBJ_SwathLength()
    print("The swath length with swath1 is " ,
        swath_length.computeCost(field, swaths1), " and with all of the swaths ",
        swath_length.computeCost(field, swaths2), "The swath length with swath2 is " ,
        swath_length.computeCost(field, swaths3))


    # RP(Route Planner) objective functions
    # Distance with turns     
    line1 = f2c.LineString(f2c.VectorPoint([f2c.Point(0.0, 0.0), f2c.Point(0.0, 1.0)]))
    swath1 = f2c.Swath(line1)
    line2 = f2c.LineString(f2c.VectorPoint([f2c.Point(1.0, 1.0), f2c.Point(1.0, 0.0)]))
    swath2 = f2c.Swath(line2)
    swaths_path = f2c.Swaths()
    swaths_path.push_back(swath1)
    swaths_path.push_back(swath2)
    robot = f2c.Robot(2.0, 3.0)
    robot.setMinTurningRadius(0.5)
    complete_length = f2c.OBJ_CompleteTurnPathObj_Dubins(robot)
    print("The complete length is: ", complete_length.computeCost(swaths_path),
    " =~= ", 1 + 1 + math.pi/2.0)


    # Direct distance without turns
    direct_dist = f2c.OBJ_DirectDistPathObj()
    print("The aproximated length is: ", direct_dist.computeCost(swaths_path))


    # PP(Path Planner) objective functions
    # Path length
    path = f2c.Path()
    path.appendSwath(swaths_path.at(0), 1)
    path.appendSwath(swaths_path.at(1), 1)

    path_length = f2c.OBJ_PathLength()
    print("The path length is: ", path_length.computeCost(path))

    f2c.Visualizer.figure()
    f2c.Visualizer.plot(swaths_path.at(0))
    f2c.Visualizer.plot(swaths_path.at(1))
    #f2c.Visualizer.plot(total_field)
    #f2c.Visualizer.plot(field)
    #f2c.Visualizer.plot(swaths1)
    #f2c.Visualizer.plot(swaths2)
    #f2c.Visualizer.plot(swaths3)
    f2c.Visualizer.show()



if __name__ == "__main__":
    main()