import cv2
import mediapipe as mp
from math import dist

class HandGestureRecognition:
    def __init__(self, robot = None):
        self.cap = cv2.VideoCapture(0)
        if robot:
            self._robot_input_system = robot.input_system
        else:
            self._robot_input_system = None
        self.mypoints = []
        self.results = None

    def orientation(self, coordinate_landmark_0, coordinate_landmark_9):
        """
        Determines the orientation of a line connecting two landmarks in a 2D space.

        This function calculates the orientation of the line formed by two landmarks (points)
        based on their x and y coordinates. The orientation can be either 'Left', 'Right', 'Up', or 'Down',
        relative to the first landmark.

        Parameters:
            coordinate_landmark_0 (tuple): The (x, y) coordinates of the first landmark.
            coordinate_landmark_9 (tuple): The (x, y) coordinates of the second (9th) landmark.

        Returns:
            str: The orientation of the line from the first landmark to the second. 
                Possible values are 'Left', 'Right', 'Up', or 'Down'.
        """

        # Extract x and y coordinates from both landmarks
        x0, y0 = coordinate_landmark_0[0], coordinate_landmark_0[1]
        x9, y9 = coordinate_landmark_9[0], coordinate_landmark_9[1]
        
        # Calculate the slope (m) of the line connecting the landmarks
        # If the x-coordinates are almost equal, set a large slope value to indicate a vertical line
        if abs(x9 - x0) < 0.05:
            m = 1000000000  # Arbitrarily large number to represent near-vertical slope
        else:
            # Calculate the absolute slope to determine the general orientation (ignoring direction)
            m = abs((y9 - y0) / (x9 - x0))

        # Determine the orientation based on the slope
        if 0 <= m <= 1:
            # Horizontal orientation, decide between 'Left' or 'Right'
            return "Right" if x9 > x0 else "Left"
        elif m > 1:
            # Vertical orientation, decide between 'Up' or 'Down'
            # Note: y decreases upwards in many graphical coordinate systems
            return "Up" if y9 > y0 else "Down"

            
    def finger(self, x, y, z):
        """
        Analyzes the state of a finger based on hand landmarks.

        This function uses the landmarks of a hand (identified by x and y) to determine
        whether a finger is closed or to return the coordinates of the tip of the finger.

        Parameters:
            x (int): The landmark index representing the mid part of the finger.
            y (int): The landmark index representing the tip of the same finger.
            z (int): A flag indicating the mode of operation; 0 for checking if the finger is closed,
                    1 for returning the coordinates of the finger tip.

        Returns:
            If z is 0: Returns an integer representing which finger is closed.
            If z is 1: Returns a tuple (x, y) of the coordinates of the finger tip.
            None: If the landmarks are not available or in case of an exception.
        """
        # Check if hand landmarks are available
        if self.results.multi_hand_landmarks is not None:
            try:
                # Extracting normalized coordinates of palm and the specified finger landmarks
                landmarks = self.results.multi_hand_landmarks[-1].landmark
                p0x, p0y = [float(str(landmarks[0]).split('\n')[i].split(" ")[1]) for i in [0, 1]]
                pmidx, pmidy = [float(str(landmarks[int(x)]).split('\n')[i].split(" ")[1]) for i in [0, 1]]
                ptopx, ptopy = [float(str(landmarks[int(y)]).split('\n')[i].split(" ")[1]) for i in [0, 1]]

                # Calculate distances from the palm to the mid and tip of the finger
                dmid, dtop = dist([p0x, p0y], [pmidx, pmidy]), dist([p0x, p0y], [ptopx, ptopy])
                
                # Check if the finger is closed based on distances
                if z == 0:
                    if dmid > dtop:
                        return {7: 1, 11: 2, 15: 3, 19: 4}.get(x, None)

                # Return the coordinates of the finger tip
                elif z == 1:
                    return int(1280 * ptopx), int(720 * ptopy)
                    
            except Exception as e:
                print(f"Error in finger function: {e}")
                # In case of an exception, the function will return None
                pass

    def x_coordinate(self, landmark):
        """
        Retrieves the x-coordinate of a specified hand landmark.

        This function extracts the x-coordinate of a given landmark index from the hand tracking
        data. The landmark index should be within the range of detected landmarks (typically 0 to 20).

        Parameters:
            landmark (int): The index of the landmark whose x-coordinate is to be retrieved.

        Returns:
            float: The x-coordinate (normalized) of the specified landmark.
        """
        # Ensure the hand landmarks are available
        if self.results.multi_hand_landmarks:
            # Extract and return the x-coordinate of the specified landmark
            return float(str(self.results.multi_hand_landmarks[-1].landmark[int(landmark)]).split('\n')[0].split(" ")[1])
        else:
            return None  # Return None if landmarks are not available

    def y_coordinate(self, landmark):
        """
        Retrieves the y-coordinate of a specified hand landmark.

        This function extracts the y-coordinate of a given landmark index from the hand tracking
        data. The landmark index should be within the range of detected landmarks (typically 0 to 20).

        Parameters:
            landmark (int): The index of the landmark whose y-coordinate is to be retrieved.

        Returns:
            float: The y-coordinate (normalized) of the specified landmark.
        """
        # Ensure the hand landmarks are available
        if self.results.multi_hand_landmarks:
            # Extract and return the y-coordinate of the specified landmark
            return float(str(self.results.multi_hand_landmarks[-1].landmark[int(landmark)]).split('\n')[1].split(" ")[1])
        else:
            return None  # Return None if landmarks are not available

    def draw(self, mypoints, image):
        """
        Draws lines between consecutive points in a given list on an image.

        This function iterates through a list of points, drawing a line on a global image
        between each consecutive pair of points. This is used to visualize paths or 
        movements, such as drawing gestures.

        Parameters:
            mypoints (list of tuples): A list of (x, y) coordinates representing points through 
                                    which lines are to be drawn. Each tuple in the list 
                                    represents a point on the image.

        Note:
            The function relies on a globally defined image variable 'image' where the lines
            will be drawn. Ensure that 'image' is defined in the global scope before calling this function.
        """
        # Iterate through each pair of consecutive points
        for i in range(len(mypoints) - 1):
            # Draw a line between the current point and the next point
            cv2.line(image, 
                    (mypoints[i][0], mypoints[i][1]),      # Current point
                    (mypoints[i+1][0], mypoints[i+1][1]),  # Next point
                    color=(255, 255, 0),                   # Line color 
                    thickness=1)                           # Line thickness
            
    def start(self):
        with mp.solutions.hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
            while self.cap.isOpened():
                success, img = self.cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    continue

                image = cv2.cvtColor(cv2.flip(img, 1), cv2.COLOR_BGR2RGB)
                image.flags.writeable = False
                results = hands.process(image)
                self.results = results

                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        mp.solutions.drawing_utils.draw_landmarks(
                            image, hand_landmarks, mp.solutions.hands.HAND_CONNECTIONS)

                    # Analyze gestures based on the landmarks
                gesture = self.analyze_gestures(results, image)
                if gesture:
                    cv2.putText(image, gesture, (500, 200), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 2)
                    #self.robot_input_system.process_command(gesture)

                cv2.imshow('MediaPipe Hands', image)
                if cv2.waitKey(5) & 0xFF == 27:  # Press 'ESC' to exit
                    break

    def stop(self):
        self.cap.release()

    def analyze_gestures(self, results, image):
        """
        Analyzes hand landmarks to detect specific gestures.

        Parameters:
            results (mediapipe.framework.formats.landmark_pb2.NormalizedLandmarkList): 
                The landmarks of the hand detected by MediaPipe.

        Returns:
            str: The identified gesture as a command or identifier.
        """
        if not results.multi_hand_landmarks:
            return None
        
        # Gesture recognition logic
        if self.finger(7, 8, 0) == 1:
            cv2.putText(image, "Index Closed", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)
        if self.finger(11, 12, 0) == 2:
            cv2.putText(image, "Middle Closed", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)
        if self.finger(15, 16, 0) == 3:
            cv2.putText(image, "Ring Closed", (500, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)
        if self.finger(19, 20, 0) == 4:
            cv2.putText(image, "Little Closed", (500, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)
        
        try:
            cv2.putText(image, self.orientation(self.finger(0, 0, 1), self.finger(9, 9, 1)),(1000,100), cv2.FONT_HERSHEY_SIMPLEX, 0.9,(0, 255, 0), 2)
        except:
            pass
            
        if self.is_thumbs_up():
            return "thumbs_up"

        # Add additional gesture recognitions here

        return None
    
    def is_thumbs_up(self):
        """
        Determines if the hand gesture is 'thumbs up'.

        Returns:
            bool: True if the gesture is 'thumbs up', False otherwise.
        """
        if self.finger(7, 8, 0) == 1 and self.finger(11, 12, 0) == 2 and self.finger(15, 16, 0) == 3 and self.finger(19, 20, 0) == 4:      #if all the fingers are closed
                if self.finger(4, 4, 1)[1] < self.finger(0, 0, 1)[1]: # y of thumb > y of palm
                    if self.orientation(self.finger(0, 0, 1), self.finger(9, 9, 1)) == "Right":
                        if self.finger(3,3,1)[0] < self.finger(5,5,1)[0]:
                            return True
        return False
    

##TEST

recognition = HandGestureRecognition()
recognition.start()
