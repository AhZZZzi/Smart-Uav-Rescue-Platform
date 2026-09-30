import cv2
import numpy as np

# Standard chessboard: 9x6 internal corners (10x7 squares)
squares_x = 10
squares_y = 7
square_size_px = 80

board_width = squares_x * square_size_px
board_height = squares_y * square_size_px

board = np.zeros((board_height, board_width), dtype=np.uint8)

for row in range(squares_y):
    for col in range(squares_x):
        if (row + col) % 2 == 0:
            y1 = row * square_size_px
            y2 = y1 + square_size_px
            x1 = col * square_size_px
            x2 = x1 + square_size_px
            board[y1:y2, x1:x2] = 255

cv2.imwrite("chessboard.png", board)
print("Chessboard saved: chessboard.png")
print(f"Size: {board_width}x{board_height} px, {squares_x}x{squares_y} squares")