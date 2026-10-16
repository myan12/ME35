import numpy as np
import matplotlib.pyplot as plt

# Define a point
point = np.array([10, 11, 1]) # this is a point at (2,3)


# we are translating this by (2,0)
translation_x = 3
translation_y = 4
translation_matrix = np.array([
    [1,0,translation_x],
    [0,1,translation_y],
    [0,0,1]])


# Create a rotation matrix for 45 degrees (pi/4 radians)
theta = np.pi/3
rotation_matrix = np.array([
    [np.cos(theta), -np.sin(theta),0],
    [np.sin(theta), np.cos(theta),0],
    [0,0,1]
])


# First translation 
translated_point = translation_matrix @ point #@ is matrix multiplication

# Then rotation
rotated_point = rotation_matrix @ translated_point

print(f"Original point: {point}")
print(f"Translated point: {translated_point}")
print(f"Rotated point: {rotated_point}")

translation_x = -3
translation_y = -4
translation_matrix = np.array([
    [1,0,translation_x],
    [0,1,translation_y],
    [0,0,1]])

translated_point2 = translation_matrix @ rotated_point #@ is matrix multiplication

# Visualization
plt.figure(figsize=(5, 5))
plt.grid(True)
plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
plt.axvline(x=0, color='k', linestyle='-', alpha=0.3)
plt.arrow(0, 0, point[0], point[1], head_width=0.1, head_length=0.1, fc='blue', ec='blue')
plt.arrow(0, 0, translated_point[0], translated_point[1], head_width=0.1, head_length=0.1, fc='red', ec='red')
plt.arrow(0, 0, rotated_point[0], rotated_point[1], head_width=0.1, head_length=0.1, fc='green', ec='green')
plt.arrow(0, 0, translated_point2[0], translated_point2[1], head_width=0.1, head_length=0.1, fc='purple', ec='purple')
plt.text(point[0], point[1], 'Original')
plt.text(translated_point[0], translated_point[1], 'Translated')
plt.text(rotated_point[0], rotated_point[1], 'Rotated')
plt.text(translated_point2[0], translated_point2[1], 'Translated Back')
plt.axis('equal')
plt.xlim(-20, 20)
plt.ylim(-20, 20)
plt.title('Translation then rotation then translation')
plt.show()