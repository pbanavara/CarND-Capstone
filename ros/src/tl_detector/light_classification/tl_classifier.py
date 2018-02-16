import numpy as np
import os
import sys
import tensorflow as tf
import time
from collections import defaultdict
from io import StringIO
from utils import label_map_util
from utils import visualization_utils as vis_util
from styx_msgs.msg import TrafficLight

MODEL_TYPE = 2 # 1 for inception, 2 for resnet

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.tl_detection = TrafficLight.UNKNOWN
        cwd = os.path.dirname(os.path.realpath(__file__))

	if MODEL_TYPE == 1:
        	PATH_TO_MODEL = cwd+'/models/frozen_sim_inception/frozen_inference_graph.pb'
		PATH_TO_LABELS = cwd+'/models/frozen_sim_inception/label_map.pbtxt'
		NUM_CLASSES = 4
	elif MODEL_TYPE == 2:
		PATH_TO_MODEL = cwd+'/models/frozen_sim_res/frozen_inference_graph.pb'
		PATH_TO_LABELS = cwd+'/models/frozen_sim_res/label_map.pbtxt'
		NUM_CLASSES = 14

        #Load model into memory
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_MODEL,'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        #Load label map
        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index=label_map_util.create_category_index(categories)

        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as self.sess:
                #define the input and output tensor for detection_graph
                self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                #get detection box
                self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                #get score
                self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0') 
        
        print "Model Loaded!"


    def load_image(self,image_path):
        """
        For testing
        """
        pass 

    def visualize(self):
        """
        for testing
        """
        pass

    def get_classification(self, image, detection_thresh=0.5):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        #image from rosbag is 1368*1096, rgb8
        #image from simulatior is 800*600 ,bgr8        
        image_np_expanded = np.expand_dims(image, axis=0)
        
        with self.detection_graph.as_default():
            (boxes, scores, classes, num) = self.sess.run(
                                                     [self.detection_boxes, self.detection_scores,
                                                      self.detection_classes,self.num_detections],
                                                      feed_dict = 
                                                      {self.image_tensor:image_np_expanded}
                                                     )        
        
        boxes = np.squeeze(boxes)
        classes = np.squeeze(classes).astype(np.int32)
        scores = np.squeeze(scores)

        for i in range(len(scores)):
            if scores is None:
                self.tl_detection = TrafficLight.UNKNOWN
            elif scores[i] >= detection_thresh :
                class_name = self.category_index[classes[i]]['name']

                if 'Green' in class_name :
                    self.tl_detection = TrafficLight.GREEN
                elif 'Yellow' in class_name:
                    self.tl_detection = TrafficLight.YELLOW
                elif 'Red' in class_name:
                    self.tl_detection = TrafficLight.RED
		else:
		    self.tl_detection = TrafficLight.UNKNOWN
        
        if np.all(scores < detection_thresh):
            self.tl_detection = TrafficLight.UNKNOWN


        return self.tl_detection


