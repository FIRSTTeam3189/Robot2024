import ntcore
import wpilib
from MLVisionExecution import MLVision
import cv2
from inference_sdk import InferenceHTTPClient
import ntcore
import wpilib

# Just assume it's from webcam 1, will change based on port on jetson/orange pi.
vid = cv2.VideoCapture(1)

CLIENT = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",
    api_key="sCJZtIgouQvZF4PmKShN"
)

class MLNetworkTable(wpilib.TimedRobot):
    def robotInit(self) -> None:

        # Model Class

        predicted_note_frames = MLVision.infer_frame()

        # Create Network Table
        inst = ntcore.NetworkTableInstance.getDefault()

        # Create Table Instance called "MLVision"
        table = inst.getTable("MLVision")

        # Publish table of an array of doubles called coordinates in which note coordinates are sent
        self.Pub = table.getDoubleTopic("coordinates").publish()

        self.test_num = 5
      

    def teleopPeriodic(self) -> None:
        
        self.Pub.set(self.test_num)

            # model = MLVision(vid, model_id="frc2024-disc/1")
            # coordinates = MLVision.extract_results(model)

            # if coordinates:
            #     coord_x = coordinates[0]['x']
            #     self.Pub.set(self.test_coords)
            #     print("Published Network Table for Coordinates")
            #     # print("X Coordinate of Note:", coordinates[0]['x'])
