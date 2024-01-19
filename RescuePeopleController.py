from GUI import GUI
from HAL import HAL
from enum import IntEnum
import cv2 as cv

class DroneState(IntEnum):
  Waiting = 0
  TakingOff = 1
  GoingToLocation = 2
  Searching = 3
  Detecting = 4
  Returning = 5
  Landed = 6

class SearchingAction(IntEnum):
  Right = 0
  Left = 1 
  Down = 2

class Drone():
  def __init__(self):
    # Position Variables
    self.max_height = 2
    self.current_location = []
    self.search_location = [44, -41, 0] # Calculated from GPS: 22.0, -20.5
    
    # On Air boolean Variables  
    self.taken_off = False
    self.landed = False
    
    # Castaway detection Variables
    self.castaways = []
    self.face_cascade = cv.CascadeClassifier(cv.data.haarcascades + 'haarcascade_frontalface_default.xml')
    self.eyes_cascade = cv.CascadeClassifier(cv.data.haarcascades + "haarcascade_eye_tree_eyeglasses.xml")
    
    # Squared search
    # ---------------->
    #                 |
    #                 v
    # <----------------
    # |
    # v
    # ---------------->
    self.searching_actions = [
      [SearchingAction.Right, 20],
      [SearchingAction.Down, 1],
      [SearchingAction.Left, 20],
      [SearchingAction.Down, 1],
      [SearchingAction.Right, 45],
      [SearchingAction.Down, 1],
      [SearchingAction.Left, 20],
      [SearchingAction.Down, 1],
      [SearchingAction.Right, 45],
      ]
    
    self.seaching_position = [0, 0, 0]  
    self.searching = False
    
    # State Variables
    self.state = DroneState.Waiting
    self.search_state = 0
    self.total_search_states = len(self.searching_actions)
    
  # ====================
  # Single use functions
  # ====================
  def set_initial_pos(self, initial_pos):
    self.current_location = initial_pos
  
  def takeoff(self):
    if not self.taken_off:
      HAL.takeoff(self.max_height)
      self.taken_off = True
      drone.increase_state()
    
  def land(self):
    if self.taken_off and not self.landed:
      HAL.land()
      self.landed = True
      drone.increase_state()
  
  # ================
  # Person detection
  # ================
  def detect_faces(self):
    image = self.get_bottom_camera()
    frame = self.get_bottom_camera()
    
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    frame_gray = cv.equalizeHist(frame_gray)
    
    people = []
         
    faces = self.face_cascade.detectMultiScale(frame_gray, 1.1, 4)
    for (x, y, w, h) in faces:
      people.append((x, y))
      cv.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            
    self.show_image(image, True)
    self.show_image(frame, False)
    
    return people
  
  def add_detected_person(self, person):
    for castaway in self.castaways:
      # Too close to a already detected castaway.
      #  Might be the same person
      if abs(castaway[0] - person[0]) < 1 and \
          abs(castaway[1] - person[1]) < 1:
        break
    
    # No close person has been found
    self.castaways.append(person)
  
  def add_detected_people(self, people):
    for person in people:
      self.add_detected_person(person)
  
  # ========
  # Movement
  # ========
  def update_position(self):
    self.current_location = HAL.get_position()
  
  def move_to(self, xy, theta=0):
    HAL.set_cmd_pos(xy[0], xy[1], self.max_height, theta)
  
  def go_to_area(self):
    self.move_to(self.search_location)
    self.state = DroneState.GoingToLocation
    
  def go_to_seach_location(self):
    self.move_to(self.seaching_position)
    
  def return_to_origin(self):
    self.move_to([0, 0], 0)
    
  def rotate(self, angle):
    self.move_to(self.current_location, angle)
 
  def rotate_and_detect(self):
    self.searching = False
    
    angle = self.current_location[2] + 0.1
    drone.rotate(angle)
    if angle > 2 * 3.14:
      angle = 0
      
    people = drone.detect_faces()
    return people
  
  def update_search_location(self):
    self.searching = True
    
    current_action = self.searching_actions[self.search_state]
    next_location = self.current_location
    
    if current_action[0] == SearchingAction.Right:
      next_location[0] += 1
      
    elif current_action[0] == SearchingAction.Left:
      next_location[0] -= 1
    
    elif current_action[0] == SearchingAction.Down:
      next_location[1] -= 1
      
    self.seaching_position = next_location
    
    # Movements are done 1 by 1
    if current_action[1] > 1:
      self.searching_actions[self.search_state][1] -= 1
    else: 
      self.search_state += 1
  
  # =============
  # Reached zones
  # =============
  def reached(self, xy, theta=None):
    position_ok = (abs(self.current_location[0] - xy[0]) < 0.2) and \
                  (abs(self.current_location[1] - xy[1]) < 0.2)
                  
    if theta is not None:
      return position_ok and abs(self.current_location[2] - theta) < 0.05
    else:
      return position_ok
                
  def reached_angle(self, angle):
    return self.reached(self.current_location, angle)
  
  def reached_zone(self):
    return self.reached(self.search_location)
    
  def reached_searching_pos(self):
    return self.reached(self.seaching_position)
    
  def reached_origin(self):
    return self.reached([0, 0], 0)
  
  # =============
  # State machine
  # =============
  def get_state(self):
    return self.state
    
  def update_state(self, new_state):
    self.state = new_state
    
  def increase_state(self):
    self.state += 1
    print("Passing to next State:", DroneState(self.state))
  
  def decrease_state(self):
    self.state -= 1
    print("Returning to previous State:", DroneState(self.state))
    
  def increase_serach_state(self):
    self.search_state += 1
  
  def finished_search(self):  
    return self.search_state == self.total_search_states
  
  def is_searching(self):
    return self.searching
  
  # =================
  # Image adquisition
  # =================
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
   
##############
#    MAIN    #
##############
drone = Drone()

drone.takeoff()
drone.set_initial_pos(HAL.get_position())
drone.increase_state()

go_back_a_state = False
starting_rotation = -1
while True:
    # State 1 - Going to Shipwreck
    # ----------------------------
    if drone.get_state() == DroneState.GoingToLocation:
      drone.show_cameras()
      print("(Going to Area) Pos:", HAL.get_position())
      
      drone.go_to_area()
      if drone.reached_zone():
        drone.increase_state()
    
    # State 2 - Looking for castaways
    # -------------------------------
    elif drone.get_state() == DroneState.Searching:
      if not drone.is_searching():
        drone.update_search_location()
      
      drone.go_to_seach_location()
      print("(Searching) Pos:", HAL.get_position())
      
      # Wait until the seaching zone has been reached
      if drone.reached_searching_pos():
        drone.increase_state()
    
    # State 3 - Detecting people in current location
    # ----------------------------------------------
    elif drone.get_state() == DroneState.Detecting:
      # Detect faces
      people = drone.rotate_and_detect()
      if people != []:
        drone.add_detected_people(people)
      
      if drone.reached_angle(starting_rotation):
        if drone.finished_search():
          drone.increase_state()
        else:
          drone.decrease_state()
    
    # State 4 - Returning to origin
    # -----------------------------
    elif drone.get_state() == DroneState.Returning:
      drone.return_to_origin()
      
      if drone.reached_origin():
        print("RETURNED!!!")
        drone.increase_state()
     
    # State 5 - Landed
    # ----------------
    elif drone.get_state() == DroneState.Landed:
      print("Found this castaways:")
      exit()
    
    # Location updating
    # -----------------
    drone.update_position()
    