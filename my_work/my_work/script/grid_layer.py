import numpy as np

class GridLayer:
    def __init__(self, min_x, max_x, min_y, max_y, grid_size):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.grid_size = grid_size

        self.grid = self.create_grid()
        self.layers = self.create_layers()

    def create_grid(self):
        x_points = np.arange(self.min_x, self.max_x + self.grid_size, self.grid_size)
        y_points = np.arange(self.min_y, self.max_y + self.grid_size, self.grid_size)
        grid = [(x, y) for x in x_points for y in y_points]
        return grid

    def create_layers(self):
        layers = {}
        layer_number = 1
        current_layer = self.grid.copy()

        while current_layer:
            next_layer = []
            for point in current_layer:
                x, y = point
                if (
                    (x - self.grid_size, y) not in self.grid or
                    (x + self.grid_size, y) not in self.grid or
                    (x, y - self.grid_size) not in self.grid or
                    (x, y + self.grid_size) not in self.grid
                ):
                    layers.setdefault(layer_number, []).append(point)
                else:
                    next_layer.append(point)

            current_layer = next_layer
            layer_number += 1

        return layers

    def get_layer(self, layer_number):
        return self.layers.get(layer_number, [])

def main():
    min_x, max_x, min_y, max_y = -2.0, 2.0, -2.0, 2.0
    grid_size = 0.25

    grid_layer = GridLayer(min_x, max_x, min_y, max_y, grid_size)

    for layer_number in sorted(grid_layer.layers.keys()):
        print(f"Layer {layer_number}: {grid_layer.get_layer(layer_number)}")

if __name__ == "__main__":
    main()
