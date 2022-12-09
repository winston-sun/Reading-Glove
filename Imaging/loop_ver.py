import numpy as np
import cv2
import pytesseract # Use REnv1 btw
import matplotlib.pyplot as plt
import os
import struct


# Only necessary if images are in RGB 
def raw_to_img(HEXADECIMAL_BYTES):
    # Reformat the bytes into an image
    raw_bytes = np.array(HEXADECIMAL_BYTES, dtype="i2")
    image = np.zeros((len(raw_bytes),3), dtype=int)

    # Loop through all of the pixels and form the image
    for i in range(len(raw_bytes)):
        #Read 16-bit pixel
        pixel = struct.unpack('>h', raw_bytes[i])[0]

        #Convert RGB565 to RGB 24-bit
        r = ((pixel >> 11) & 0x1f) << 3
        g = ((pixel >> 5) & 0x3f) << 2
        b = ((pixel >> 0) & 0x1f) << 3
        image[i] = [r,g,b]

    image = np.reshape(image,(144, 176,3)) #QCIF resolution
    image = image.astype(np.uint8) # MIGHT need this, idk yet
    return image

def array_to_img(array):
    temp = np.array(array)
    image = np.reshape(temp,(120, 100)) #QCIF resolution
    image = image.astype(np.uint8) # MIGHT need this, idk yet
    return image

# Main loop 
while True:
    images = []
    last = False # TODO: Set this to true on last image
    while not last:
        raw_image = 1 # TODO: Capture image
        image = raw_to_img(raw_image)
        # Grayscale, Gaussian blur, Otsu's threshold
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (3,3), 0)
        thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
        
        images.append(image)

        # TODO: Change last value 
        last = True
    stitcher =  cv2.Stitcher.create() # cv2.createStitcher(), if opencv ver is diff
    # images is a list of images
    (status, stitched) = stitcher.stitch(images)
    

    output = pytesseract.image_to_string(gray, lang='eng', config='--psm 6')
    print(output)

    output = output.replace('\n', ' ')

    os.system("say -v 'Samantha' " + output)