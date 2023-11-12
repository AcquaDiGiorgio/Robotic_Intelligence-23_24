from GUI import GUI
from HAL import HAL
import cv2

def getImage():
  img = HAL.getImage()
  hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
  red_mask = cv2.inRange(hsv, (0, 125, 125), (30, 255, 255))
  contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  
  return img, red_mask, contours

def setSpeed(v, w):
  HAL.setV(v)
  HAL.setW(w)

def process(contours):
  M = cv2.moments(contours[0])
  
  cX, cY = 0, 0
  if M["m00"] != 0:
    cX = M["m10"] / M["m00"]
    cY = M["m01"] / M["m00"]
    
  if cX > 0:
    err = 320 - cX
    setSpeed(2, 0.01*err)
  
  return cX, cY
  
i = 0
last_cX = 0
last_cY = 0
sameC = 0

while True:
    img, red_mask, contours = getImage()  
    cX, cY = process(contours)
    GUI.showImage(red_mask)
    
    print('%d -> cX: %.2f  cY: %.2f' % (i, cX, cY))
    
    # Reseting Purpose
    if cX == last_cX and cY == last_cY:
      sameC = sameC + 1
      if sameC > 20:
        setSpeed(0, 0)
        exit()
    else:
      sameC = 0
    
    last_cX = cX
    last_cY = cY
    i = i + 1