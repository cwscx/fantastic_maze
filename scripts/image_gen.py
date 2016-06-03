import cv2
import numpy as np
import os

class ImageGen:
    def __init__(self, x, y, data):
        self.map_x = x
        self.map_y = x
        self.MAP_SHAPE = ((self.map_x * 20), (self.map_y * 20), 3)
        
        self.wall = cv2.imread("../img/wall.jpg")
        self.maze = data

    def save_image(self):
        height, width, layers = self.MAP_SHAPE
        #Creating an empty map of white spaces
        empty_map = np.zeros(self.MAP_SHAPE)
        empty_map.fill(255)
        for row in range(self.map_x):
            for col in range(self.map_y):
                new_pos_row = (row * 20)
                new_pos_col = (col * 20)

                if self.maze[row, col] == 1:
                    empty_map[new_pos_row : new_pos_row + 20, new_pos_col : new_pos_col + 20] = self.wall
        print('saving image')
        cv2.imwrite(os.path.dirname(os.path.realpath(__file__)) + "/maze.png", empty_map)
