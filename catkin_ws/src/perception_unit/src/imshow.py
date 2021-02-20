import cv2
from matplotlib import pyplot as plt

# Read image via OpenCV
import cv2
# import imageio

# img = imageio.imread('https://upload.wikimedia.org/wikipedia/en/7/7d/Lenna_%28test_image%29.png?download')
img = cv2.imread('/')
img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
#img = cv2.imread('lenna.png')

# region = cv2.selectROI('window', img)
cv2.imshow('window', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# x,y,w,h = region
# new_img = img[y:y+h,x:x+w]

# cv2.waitKey(0)
# cv2.destroyAllWindows()
