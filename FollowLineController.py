from GUI import GUI
from HAL import HAL
import cv2

verbose = True

class Robot():
  def __init__(self):
    self.prev_error = 0
    self.sum_error = 0
    
    self.targetX = 320
    self.targetY = 398
    
    self.prev_cX = 0
    self.prev_cY = 0
    
    self.iters_motionless = 0
    
    self.KP = 0.0026
    self.KD = 0.013
    self.KI = 0.000005
    
    self.v = 4.0
    self.max_w = 1.5
    
  
  def set_angular_Naive(self, w):
    HAL.setV(self.v)
    HAL.setW(w)
    
    return self.v
    
    
  def set_angular(self, w):
    v = (self.max_w - abs(w)) / self.max_w * self.v
    
    HAL.setV(v)
    HAL.setW(w)
    
    return v
    
  
  def set_speed(self, v, w):
    HAL.setV(v)
    HAL.setW(w)
    
    
  def setSpeed_cXY(self, cX, cY):
    # Error E [0, 1]
    error = self.targetX - cX
    self.sum_error += error
    
    if abs(error) < 0.05:
      self.sum_error = 0
    
    error_p = error * self.KP 
    error_i = self.sum_error * self.KI
    error_d = (error - self.prev_error) * self.KD 

    w = error_p + error_i + error_d
    
    #if w > self.max_w:
    #  w = self.max_w
    #elif w < -self.max_w:
    #  w = -self.max_w
    
    v = self.set_angular_Naive(w)
    # v = self.v - (abs(self.targetY - cY) / self.targetY) * self.v
    # self.set_speed(v, w)
    
    #v = self.set_angular(w)
    self.prev_error = error
    
    if verbose:
      print("error_p:", error_p)
      print("error_i:", error_i)
      print("error_d:", error_d)
      print("Linear speed:", v)
      print("Angular speed:", w)
    
  
  def process(self, i):
    img = HAL.getImage()
    max_x = int(img.shape[0] - img.shape[0]/2.15)
    hsv = cv2.cvtColor(img[max_x:,:,:], cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv, (0, 125, 125), (30, 255, 255))
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    cX, cY = 0, 0
    
    # Exception Prevention 
    if len(contours) != 0:
      M = cv2.moments(contours[0])
      
      if M["m00"] != 0:
        cX = M["m10"] / M["m00"]
        cY = M["m01"] / M["m00"]
        
    else:
      cX = self.prev_cX
      cY = self.prev_cY
    
    self.setSpeed_cXY(cX, cY)
      
    if verbose:
      print('%d) cX: %.2f  cY: %.2f' % (i, cX, cY))
      print()
      
    GUI.showImage(red_mask)
    
    
i = 0
rob = Robot()
while True:
    #test()
    rob.process(i)
    i += 1