# move_back_to_previous_crs.py
import fields2cover as f2c

def move_back_to_previous_crs(field, path):
    print("Transforming path back to previous CRS...")
    path_gps = f2c.Transform.transformToPrevCRS(path, field)
    f2c.Transform.transformToPrevCRS(field)
    
    ps = [path_gps.getState(i).point for i in range(path_gps.size())]
    L = f2c.LineString(f2c.VectorPoint(ps))
    
    # 시각화
    f2c.Visualizer.figure()
    f2c.Visualizer.plot(field.getCellsAbsPosition())
    f2c.Visualizer.plot(L)
    f2c.Visualizer.save("Tutorial_8_1_GPS.png")
    print("GPS visualization saved as 'Tutorial_8_1_GPS.png'.")

if __name__ == "__main__":
    from create_path import create_path
    from import_field import import_field

    file_path = '/home/adsol/costmap.xml'
    field = import_field(file_path)
    path = create_path(field)
    move_back_to_previous_crs(field, path)
