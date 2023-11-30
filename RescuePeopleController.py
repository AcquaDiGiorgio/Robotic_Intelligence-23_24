from GUI import GUI
from HAL import HAL
from enum import IntEnum
import cv2 as cv
# Enter sequential code!

class DroneState(IntEnum):
  Waiting = 0
  TakingOff = 1
  GoingToLocation = 2
  Searching = 3
  Returning = 4
  Landing = 5

class Drone():
  def __init__(self):
    self.max_height = 2
    self.people = []
    self.search_location = [40, -40, 0]
    self.current_location = []
    self.search_state = 0
    self.taken_off = False
    
    self.face_cascade = cv.CascadeClassifier(cv.data.haarcascades + 'haarcascade_frontalface_default.xml')
    self.eyes_cascade = cv.CascadeClassifier(cv.data.haarcascades + "haarcascade_eye_tree_eyeglasses.xml")
    
    self.state = DroneState.Waiting
      
  # Single use functions
  def set_initial_pos(self, initial_pos):
    self.current_location = initial_pos
  
  def takeoff(self):
    if not self.taken_off:
      HAL.takeoff(self.max_height)
      self.taken_off = True
    
  def land(self):
    if self.taken_off:
      HAL.land()
      self.taken_off = False
  
  # Continuous usage functions  
  def update_position(self):
    self.current_location = HAL.get_position()
      
  def move_to(self, x, y, theta):
    HAL.set_cmd_pos(x, y, self.max_height, theta)
  
  def go_to_area(self, yaw):
    self.move_to(self.search_location[0], self.search_location[1], yaw)
    self.state = DroneState.GoingToLocation
  
  def search_area(self, style):
    pass
  
  def return_origin(self):
    self.move_to(0, 0, 0)

  # Image adquisition
  def get_bottom_camera(self):
    return HAL.get_ventral_image()
    
  def get_frontal_camera(self):
    return HAL.get_frontal_image()
    
  def get_cameras_info(self):
    return self.get_frontal_camera(), self.get_bottom_camera()

  def show_cameras(self):
    im1, im2 = self.get_cameras_info()
    GUI.showImage(im1)
    GUI.showLeftImage(im2)
    
  def show_image(self, image,  on_left=False):
    if on_left:
      GUI.showLeftImage(image)
    else:
      GUI.showImage(image)
    
  def detect_faces(self):
    image = self.get_bottom_camera()
    frame = self.get_bottom_camera()
    
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    frame_gray = cv.equalizeHist(frame_gray)
    #-- Detect faces
    #faces = self.face_cascade.detectMultiScale(frame_gray)
    #for (x,y,w,h) in faces:
    #    center = (x + w//2, y + h//2)
    #    frame = cv.ellipse(frame, center, (w//2, h//2), 0, 0, 360, (255, 0, 255), 4)
    #    faceROI = frame_gray[y:y+h,x:x+w]
    #    #-- In each face, detect eyes
    #    eyes = self.eyes_cascade.detectMultiScale(faceROI)
    #    for (x2,y2,w2,h2) in eyes:
    #        eye_center = (x + x2 + w2//2, y + y2 + h2//2)
    #        radius = int(round((w2 + h2)*0.25))
    #        frame = cv.circle(frame, eye_center, radius, (255, 0, 0 ), 4)
            
    faces = self.face_cascade.detectMultiScale(frame_gray, 1.1, 4)
    for (x, y, w, h) in faces:
      cv.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            
    self.show_image(image, True)
    self.show_image(frame, False)
  
  def get_state(self):
    return self.state
  
  def rotate(self, angle):
    self.move_to(self.current_location[0], self.current_location[1], angle)
 
  def reached(self, x, y, theta=None):
    position_ok = (abs(self.current_location[0] - x) < 0.2) and \
                  (abs(self.current_location[1] - y) < 0.2)
                  
    if theta is not None:
      return position_ok and abs(self.current_location[2] - theta) < 0.1
    else:
      return position_ok
                
  
  def reached_zone(self):
    return self.reached(self.search_area[0], self.search_area[1])
    
  def update_state(self, new_state):
    self.state = new_state
    
  def increase_state(self):
    self.state += 1
    
drone = Drone()

drone.takeoff()
drone.set_initial_pos(HAL.get_position())
drone.increase_state()

angle = 0
pi = 3.14

while True:
    if drone.get_state() == DroneState.GoingToLocation:
      drone.show_cameras()
      print("Going to Area:", HAL.get_position())
      drone.go_to_area(HAL.get_yaw())
      if drone.reached_zone():
        print("REACHED!!!")
        drone.increase_state()
      
    elif drone.get_state() == DroneState.Searching:
      drone.detect_faces()
      drone.rotate(angle)
      angle += 0.1
      if angle > 2 * pi:
        angle = 0
      
    drone.update_position()
    