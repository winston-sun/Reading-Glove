import numpy as np
import cv2
import pytesseract # Use REnv1 btw
import matplotlib.pyplot as plt
import os
import struct
import serial
import serial.tools.list_ports
from time import sleep

import numpy as np
import cv2

'''
Following functions were taken from
https://stackoverflow.com/questions/45612933/panorama-stitching-for-text/45639406#45639406
'''

H_templ_ratio = 0.3 # H_templ_ratio: horizontal ratio of the input that we will keep to create a template

def genTemplate(img): 
    global H_templ_ratio
    # we get the image's width and height
    h, w = img.shape[:2]
    # we compute the template's bounds
    x1 = int(float(w)*(1-H_templ_ratio))
    y1 = 0
    x2 = w
    y2 = h
    return(img[y1:y2,x1:x2]) # and crop the input image

def mat2Edges(img): # applies a Canny filter to get the edges
    edged = cv2.Canny(img, 100, 200)
    return(edged)

def addBlackMargins(img, top, bottom, left, right): # top, bottom, left, right: margins width in pixels
    h, w = img.shape[:2]
    result = np.zeros((h+top+bottom, w+left+right, 3), np.uint8)
    result[top:top+h,left:left+w] = img
    return(result)

# return the y_offset of the first image to stitch and the final image size needed
def calcFinalImgSize(imgs, loc):
    global V_templ_ratio, H_templ_ratio
    y_offset = 0
    max_margin_top = 0; max_margin_bottom = 0 # maximum margins that will be needed above and bellow the first image in order to stitch all the images into one mat
    current_margin_top = 0; current_margin_bottom = 0

    h_init, w_init = imgs[0].shape[:2]
    w_final = w_init
    
    for i in range(0,len(loc)):
        h, w = imgs[i].shape[:2]
        h2, w2 = imgs[i+1].shape[:2]
        # we compute the max top/bottom margins that will be needed (relatively to the first input image) in order to stitch all the images
        current_margin_top += loc[i][1] # here, we assume that the template top-left corner Y-coordinate is 0 (relatively to its original image)
        current_margin_bottom += (h2 - loc[i][1]) - h
        if(current_margin_top > max_margin_top): max_margin_top = current_margin_top
        if(current_margin_bottom > max_margin_bottom): max_margin_bottom = current_margin_bottom
        # we compute the width needed for the final result
        x_templ = int(float(w)*H_templ_ratio) # x-coordinate of the template relatively to its original image
        w_final += (w2 - x_templ - loc[i][0]) # width needed to stitch all the images into one mat

    h_final = h_init + max_margin_top + max_margin_bottom
    return (max_margin_top, h_final, w_final)

# match each input image with its following image (1->2, 2->3) 
def matchImages(imgs, templates_loc):
    for i in range(0,len(imgs)-1):
        template = genTemplate(imgs[i])
        template = mat2Edges(template)
        h_templ, w_templ = template.shape[:2]
        # Apply template Matching
        margin_top = margin_bottom = h_templ; margin_left = margin_right = 0
        img = addBlackMargins(imgs[i+1],margin_top, margin_bottom, margin_left, margin_right) # we need to enlarge the input image prior to call matchTemplate (template needs to be strictly smaller than the input image)
        img = mat2Edges(img)
        res = cv2.matchTemplate(img,template,cv2.TM_CCOEFF) # matching function
        _, _, _, templ_pos = cv2.minMaxLoc(res) # minMaxLoc gets the best match position
        # as we added margins to the input image we need to subtract the margins width to get the template position relatively to the initial input image (without the black margins)
        rectified_templ_pos = (templ_pos[0]-margin_left, templ_pos[1]-margin_top) 
        templates_loc.append(rectified_templ_pos)
        print("max_loc", rectified_templ_pos)

def stitchImages(imgs, templates_loc):
    y_offset, h_final, w_final = calcFinalImgSize(imgs, templates_loc) # we calculate the "surface" needed to stitch all the images into one mat (and y_offset, the Y offset of the first image to be stitched) 
    result = np.zeros((h_final, w_final, 3), np.uint8)

    #initial stitch
    h_init, w_init = imgs[0].shape[:2]
    result[y_offset:y_offset+h_init, 0:w_init] = imgs[0]
    origin = (y_offset, 0) # top-left corner of the last stitched image (y,x)
    # stitching loop
    for j in range(0,len(templates_loc)):
        h, w = imgs[j].shape[:2]
        h2, w2 = imgs[j+1].shape[:2]
        # we compute the coordinates where to stitch imgs[j+1]
        y1 = origin[0] - templates_loc[j][1]
        y2 = origin[0] - templates_loc[j][1] + h2
        x_templ = int(float(w)*(1-H_templ_ratio)) # x-coordinate of the template relatively to its original image's right side
        x1 = origin[1] + x_templ - templates_loc[j][0]
        x2 = origin[1] + x_templ - templates_loc[j][0] + w2
        result[y1:y2, x1:x2] = imgs[j+1] # we copy the input image into the result mat
        origin = (y1,x1) # we update the origin point with the last stitched image

    return(result)

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
    image = np.reshape(temp,(100, 220)) #QCIF resolution
    image = image.astype(np.uint8) # MIGHT need this, idk yet
    return image



sPort = '/dev/cu.usbmodem14201' 
aSerialData = serial.Serial(sPort, 115200)
flag = False

# Defining main function
def main():
    # Main loop 
    captured_images = 0
    images = []

    while True:
        flag = False
        temp = False
        # sleep(2) # Change this to wait till 12000 
        # if (aSerialData.inWaiting()==1):
        #     temp = True
            
        if (aSerialData.inWaiting()>0 or temp):
            
            sData = aSerialData.readline()
            sData = (str(sData).split(","))
            if (len(sData) != 220 * 100 + 1):
                if sData == ["b'Image capture...\\r\\n'"]:
                    print("----------Move to next pos---------------")
                print(sData)
                continue
            print("Image Capturing...")
            # os.system("say -v 'Samantha' " + "ok")
            temp_processed = sData[0:220 * 100]
            temp_processed[0] = temp_processed[0][2:]
            temp_processed = [eval(i) + 128 for i in temp_processed] 
            image = array_to_img(temp_processed)
            
            captured_images += 1
            flipped = image.T
            colored = cv2.cvtColor(flipped, cv2.COLOR_GRAY2BGR)

            images.append(colored)
            plt.figure()
            plt.imshow(image, cmap='gray')
            plt.show()
            plt.imsave("big_test" + str(captured_images) + ".jpg", image)

            if captured_images % 2 == 0:
                captured_images = 0
                print("Image Processing...")
                flag = True
                # input images

                
                templates_loc = [] # templates location
                matchImages(images, templates_loc)
                x = stitchImages(images, templates_loc)
                plt.figure()
                y = cv2.cvtColor(x, cv2.COLOR_BGR2GRAY).T
                plt.imshow(y, cmap='gray')
                plt.show()

                output = pytesseract.image_to_string(y, lang='eng', config='--psm 6')
                print(output)
                output = output.replace('\n', ' ')
                new_str = ""
                for i in output:
                    if i.isalnum() or i == " ":
                        new_str += i

                os.system("say -v 'Samantha' " + new_str)
                # print(output)

                # stitcher =  cv2.Stitcher.create() # cv2.createStitcher(), if opencv ver is diff
                # images is a list of images

                # (status, stitched) = stitcher.stitch(images)
                # print("STATUS")
                # print(status)
                # plt.imsave('stitched.jpg', stitched)

                # NOT USIGN ALL OF THESE 
                # gray = cv2.cvtColor(stitched, cv2.COLOR_BGR2GRAY)
                # blur = cv2.GaussianBlur(gray, (3,3), 0)
                # thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

                # output = pytesseract.image_to_string(gray, lang='eng', config='--psm 6')
                # print(output)
                # output = output.replace('\n', ' ')
                # os.system("say -v 'Samantha' " + output)
                
                images = []
                # return

        # images = []
        # last = False # TODO: Set this to true on last image
        # while not last:
        #     raw_image = 1 # TODO: Capture image
        #     image = raw_to_img(raw_image)
        #     # Grayscale, Gaussian blur, Otsu's threshold
        #     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #     blur = cv2.GaussianBlur(gray, (3,3), 0)
        #     thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
            
        #     images.append(image)

        #     # TODO: Change last value 
        #     last = True
        # stitcher =  cv2.Stitcher.create() # cv2.createStitcher(), if opencv ver is diff
        # # images is a list of images
        # (status, stitched) = stitcher.stitch(images)
        

        # output = pytesseract.image_to_string(gray, lang='eng', config='--psm 6')
        # print(output)

        # output = output.replace('\n', ' ')

        # os.system("say -v 'Samantha' " + output)
  
  
# Using the special variable 
# __name__
if __name__=="__main__":
    main()
