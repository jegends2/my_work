import fields2cover as f2c


# Headland generator
def main():

    rand = f2c.Random(42)
    field = rand.generateRandField(1e4, 5)
    cells = field.getField()
    robot = f2c.Robot(2.0, 26.0);rand = f2c.Random(42)


    const_hl = f2c.HG_Const_gen()
    no_hl = const_hl.generateHeadlands(cells, 3.0 * robot.getWidth())

    f2c.Visualizer.figure()
    f2c.Visualizer.plot(cells)
    f2c.Visualizer.plot(no_hl)
    f2c.Visualizer.show()
    
if __name__ == "__main__":
    main()