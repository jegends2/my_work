# create_path.py
import fields2cover as f2c
import math

def create_path(field):
    print("Creating path...")
    
    robot = f2c.Robot(2.0, 6.0)
    robot.setMinTurningRadius(2)

    const_hl = f2c.HG_Const_gen()
    no_hl = const_hl.generateHeadlands(field.getField(), 3.0 * robot.getWidth())
    
    bf = f2c.SG_BruteForce()
    swaths = bf.generateSwaths(math.pi, robot.getCovWidth(), no_hl.getGeometry(0))
    
    snake_sorter = f2c.RP_Snake()
    swaths = snake_sorter.genSortedSwaths(swaths)
    
    path_planner = f2c.PP_PathPlanning()
    dubins = f2c.PP_DubinsCurves()
    path = path_planner.planPath(robot, swaths, dubins)
    
    # 시각화
    f2c.Visualizer.figure()
    f2c.Visualizer.plot(field)
    f2c.Visualizer.plot(no_hl)
    f2c.Visualizer.plot(path)
    f2c.Visualizer.save("Tutorial_8_1_UTM.png")
    print("Path visualization saved as 'Tutorial_8_1_UTM.png'.")
    
    return path

if __name__ == "__main__":
    from transform_to_utm import transform_to_utm
    from import_field import import_field

    file_path = '/home/adsol/costmap.xml'
    field = import_field(file_path)
    transform_to_utm(field)
    path = create_path(field)
