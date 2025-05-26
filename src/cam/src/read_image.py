
import cv2 as cv
import sys
import numpy as np

img = cv.imread(cv.samples.findFile("coins.jpg"))

if img is None:
    sys.exit("Could not read the image.")

cv.imshow("Display window", img)

# grayscale 
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

# Gaussian blur to reduce noise
blurred = cv.GaussianBlur(gray, (5, 5), 0)# image, kernel size (width, height), sigmaX

# simple thresholding to get binary image
ret, binary = cv.threshold(blurred, 127, 255, cv.THRESH_BINARY)# image, threshold value, max value, threshold type

# Alternative: Otsu's thresholding (automatically determines optimal threshold)
ret_otsu, binary_otsu = cv.threshold(blurred, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)

# erosion and dilation
# Create a kernel for morphological operations
kernel = np.ones((5, 5), np.uint8)

# Erosion (shrinks bright regions, removes small white noise)
eroded = cv.erode(binary_otsu, kernel, iterations=1)

# Dilation (expands bright regions, closes small holes)
dilated = cv.dilate(binary_otsu, kernel, iterations=1)

 # 找輪廓
contours, _ = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
# cv2.RETR_EXTERNAL：只找外部輪廓（避免內部小區域干擾）。
# cv2.CHAIN_APPROX_SIMPLE：儲存輪廓的關鍵點，而不儲存所有邊界點，減少計算量

 # 過濾較小的區域，避免誤判
min_area = 500  # 設定最小面積閾值
valid_contours = [cnt for cnt in contours if cv.contourArea(cnt) > min_area] # 保留足夠大的區域

   # 確保至少有一個有效輪廓
if valid_contours:
    x, y, w, h = cv.boundingRect(np.vstack(valid_contours)) # 計算最小外接矩形
    cropped = binary[y:y+h, x:x+w]

# Resize 
display_width = 800  
display_height = 600 

original_resized = cv.resize(img, (display_width, display_height))
binary_resized = cv.resize(binary_otsu, (display_width, display_height))
eroded_resized = cv.resize(eroded, (display_width, display_height))
dilated_resized = cv.resize(dilated, (display_width, display_height))

# Display 
cv.imshow("Original Image", original_resized)
cv.imshow("Binary Image (Otsu)", binary_resized)
cv.imshow("Eroded Image", eroded_resized)
cv.imshow("Dilated Image", dilated_resized)

# Wait for a key press and close all windows
cv.waitKey(0)
cv.destroyAllWindows()

# is 's' pressed, save image
if k == ord("s"):
    cv.imwrite("binary.jpg", binary_otsu)
    cv.imwrite("eroded.jpg", eroded)
    cv.imwrite("dilated.jpg", dilated)

