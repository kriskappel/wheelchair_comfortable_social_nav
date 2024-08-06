import cv2
import numpy as np
import os


def extract_purple_and_overlay(image, overlay_image, output_path):
    # Load the image containing the map and purple line
    # image = cv2.imread(image_path)
    
    # Convert the image to the HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Define the range for the purple color in HSV space
    lower_purple = np.array([130, 50, 50])
    upper_purple = np.array([160, 255, 255])
    
    # Create a mask for the purple color
    mask = cv2.inRange(hsv, lower_purple, upper_purple)
    
    # Extract the purple parts using the mask
    purple_parts = cv2.bitwise_and(image, image, mask=mask)
    
    # Load the image to overlay the purple parts onto
    # overlay_image = cv2.imread(overlay_image_path)
    
    # Ensure the overlay image is the same size as the original image
    overlay_image_resized = cv2.resize(overlay_image, (image.shape[1], image.shape[0]))
    
    # Create an inverse mask of the purple parts
    inverse_mask = cv2.bitwise_not(mask)
    
    # Use the inverse mask to make the areas of the purple parts transparent on the overlay image
    overlay_bg = cv2.bitwise_and(overlay_image_resized, overlay_image_resized, mask=inverse_mask)
    
    # Add the purple parts to the overlay image
    result = cv2.add(overlay_bg, purple_parts)
    
    # Save the result
    return result

def process_tif_images():
    output_path = 'teste.png'
    # Get the list of all files in the current directory
    files = os.listdir('.')
    
    # Filter out the files that end with .tif
    tif_files = [file for file in files if file.lower().endswith('.tif')]
    
    overlay_image=None
    first_flag = True
    # Loop through the list of .tif files and open each one
    for tif_file in tif_files:

        if first_flag:
            overlay_image = cv2.imread(tif_file)
            first_flag = False
        # Load the image
        else:
            image = cv2.imread(tif_file)
            overlay_image = extract_purple_and_overlay(image, overlay_image, output_path)
            

    # Save the result
    cv2.imwrite(output_path, overlay_image)
            

# # Define the paths to your images and output
# input_image_path = 'anglo_nav_19:17:15.tif' #imagem que vai remover a linha
# overlay_image_path = 'corridor_comfort_nav_03:49:19.tif' #image q vai ficar de base
# output_image_path = 'teste.png'

# # Call the function
# extract_purple_and_overlay(input_image_path, overlay_image_path, output_image_path)




# Call the function to process .tif images
process_tif_images()
