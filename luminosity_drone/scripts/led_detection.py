# import the necessary packages
from imutils import contours
from skimage import measure
import numpy as np
import imutils
import cv2

# load the image, 
image = cv2.imread('led.jpg', 1)

# convert it to grayscale, and blur it

# threshold the image to reveal light regions in the blurred image


# perform a series of erosions and dilations to remove any small blobs of noise from the thresholded image

# perform a connected component analysis on the thresholded image, then initialize a mask to store only the "large" components

# loop over the unique components

	# if this is the background label, ignore it

	# otherwise, construct the label mask and count the number of pixels 

	# if the number of pixels in the component is sufficiently large, then add it to our mask of "large blobs"
	
# find the contours in the mask, then sort them from left to right

# loop over the contours

# Initialize lists to store centroid coordinates and area

# Loop over the contours


    # Calculate the area of the contour
    

    # Draw the bright spot on the image


    # Append centroid coordinates and area to the respective lists

# Save the output image as a PNG file
cv2.imwrite("led_detection_results.png", image)

# Open a text file for writing
with open("led_detection_results.txt", "w") as file:
    # Write the number of LEDs detected to the file
    file.write(f"No. of LEDs detected: {a}\n")
    # Loop over the contours
    
        # Write centroid coordinates and area for each LED to the file
        file.write(f"Centroid #{i + 1}: {centroid}\nArea #{i + 1}: {area}\n")
# Close the text file
file.close()
