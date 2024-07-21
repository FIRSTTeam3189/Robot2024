from inference_sdk import InferenceHTTPClient
import cv2

vid = cv2.VideoCapture(0)

CLIENT = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",
    api_key="sCJZtIgouQvZF4PmKShN"
)


class MLVision:
    def __init__(self, vid, model_id="frc2024-disc/1"):
        self.vid = vid
        self.model_id = model_id
        self.success, self.frame = self.vid.read()
        if not self.success:
            raise Exception("Video Capture Failed")

        self.result = self.infer_frame()

    def infer_frame(self):
        return CLIENT.infer(self.frame, model_id=self.model_id)

    def extract_results(self):
        if 'predictions' in self.result:
            results = []
            for prediction in self.result['predictions']:
                if all(k in prediction for k in ('x', 'y', 'width', 'height', 'class', 'confidence')):
                    x = int(prediction['x'])
                    y = int(prediction['y'])
                    width = int(prediction['width'])
                    height = int(prediction['height'])
                    label = prediction['class']
                    confidence = prediction['confidence']
                    results.append(
                        {'x': x, 'y': y, 'width': width, 'height': height, 'label': label, 'confidence': confidence})
            return results
        else:
            return []

    def get_bbox(self, x, y, width, height):
        x0 = x - width // 2
        y0 = y - height // 2
        x1 = x + width // 2
        y1 = y + height // 2
        return x0, y0, x1, y1
