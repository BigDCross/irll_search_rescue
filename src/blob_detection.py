import cv2
import cv2.cv as cv
import numpy as np
from copy import deepcopy

class Vision_Processor ():
    def __init__(self):
        self.images = {}
    
        self.settings = {}

        self.outputs = {}

        self.outputted_images = ["Src", "Color_Filter", "Contours"]
    
    def process_image (self, image):
        self.images["Src"] = deepcopy (image) 
    
        self.images["MFilter"] = cv2.medianBlur (self.images["Src"], 9)

        self.images["HSV"] = cv2.cvtColor (self.images["MFilter"], cv2.COLOR_BGR2HSV)

        self.images["Color_Filter"] = cv2.inRange (self.images["HSV"], (160, 50, 50), (180, 230, 230))

        self.outputs["Contours"], heirarchy = cv2.findContours (deepcopy (self.images["Color_Filter"]), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cv2.drawContours (self.images["Src"], self.outputs["Contours"], -1, (0, 0, 0), thickness=5)

        self.outputs["Big_Contours"] = []

        for i in self.outputs["Contours"]:
            area = cv.ContourArea (cv.fromarray (i))
            moment = cv2.moments (i)
            if area > 1000:
                self.outputs["Big_Contours"].append (i)
                print area,
                print " ",
                print moment
		return 1
        return 0

    def show_images (self):
        if self.outputted_images:
            for key in self.outputted_images:
                try:
                    cv2.imshow (key, self.images[key])
                except:
                    pass
        else:
            for key in self.images.keys():
                try:
                    cv2.imshow (key, self.images[key])
                except:
                    pass

def main ():
    #try:
        #cap = cv2.VideoCapture (1)
    #except:
        #cap = cv2.VideoCapture (0)
    cap = cv2.VideoCapture (0)

    processor = Vision_Processor ()
    while cap.isOpened():
        ret, image = cap.read ()

        processor.process_image (image)
        processor.show_images ()
        
        key = cv2.waitKey (20)
        if key == 1048603 or key == 27: # idk why it switches...wait...windows or linux?
            cv2.destroyAllWindows()
            cap.release()

if __name__=="__main__":
    main ()
