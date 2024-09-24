import fields2cover as f2c

def main():
    # Define the swaths
    line1 = f2c.LineString(f2c.VectorPoint([f2c.Point(0.0, 0.0), f2c.Point(0.0, 1.0)]))
    line2 = f2c.LineString(f2c.VectorPoint([f2c.Point(1.0, 1.0), f2c.Point(1.0, 0.0)]))
    swath1 = f2c.Swath(line1)
    swath2 = f2c.Swath(line2)
    swaths_path = f2c.Swaths()
    swaths_path.push_back(swath1)
    swaths_path.push_back(swath2)
    
    # Define the path
    path = f2c.Path()
    path.appendSwath(swaths_path.at(0), 1)
    path.appendSwath(swaths_path.at(1), 1)
    
    f2c.Visualizer.figure()
    f2c.Visualizer.plot(swath1)
    f2c.Visualizer.plot(swath2)
    f2c.Visualizer.plot(swaths_path)
    f2c.Visualizer.show()

if __name__ == "__main__":
    main()