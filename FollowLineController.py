from GUI import GUI
from HAL import HAL
import cv2

class Robot():
  def __init__(self):
    self.prev_error = 0
    
    self.prev_cX = 0
    self.prev_cY = 0
    self.iters_motionless = 0
    
    self.KP = 0.05
    self.KD = 0.025
    
    self.v = 2
    self.max_w = 1
    
  
  def set_angular_Naive(self, w):
    HAL.setV(self.v)
    HAL.setW(w)
    
    return self.v
    
    
  def set_angular(self, w):
    v = (self.max_w - abs(w)) * self.v
    
    HAL.setV(v)
    HAL.setW(w)
    
    return v
    
    
  # cY = 398.29
  def setSpeed_cXY(self, cX, cY):
    error_p = 320 - cX
    error_d = error_p - self.prev_error
    
    self.prev_error = error_p
    w = 0.02 * (error_p * self.KP + error_d * self.KD)
    
    if w > self.max_w:
      w = self.max_w
    elif w < -self.max_w:
      w = -self.max_w
    
    print("error_p:", error_p)
    print("error_d:", error_d)
    print("previous:", self.prev_error)
    
    v = self.set_angular(w)
    print("Linear speed:", v, "Angular speed:", w)
    
  
  def process(self, i):
    img = HAL.getImage()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv, (0, 125, 125), (30, 255, 255))
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    M = cv2.moments(contours[0])
    
    cX, cY = 0, 0
    if M["m00"] != 0:
      cX = M["m10"] / M["m00"]
      cY = M["m01"] / M["m00"]
    
    self.setSpeed_cXY(cX, cY)
      
    # if cX > 0:
    #   err = 320 - cX
    #   self.setSpeed(2, 0.01*err)
    
    print('%d) cX: %.2f  cY: %.2f' % (i, cX, cY))
    print()
    GUI.showImage(red_mask)
    
    self.check_motion(cX, cY)
    
    
  def check_motion(self, cX, cY):
    if cX == self.prev_cX and cY == self.prev_cY:
      self.iters_motionless += 1
      
      if self.iters_motionless > 20:
        exit()
    
    else:
      self.iters_motionless = 0
    
    self.prev_cX = cX
    self.prev_cY = cY
    
    
i = 0
rob = Robot()

while True:
    rob.process(i)
    i += 1