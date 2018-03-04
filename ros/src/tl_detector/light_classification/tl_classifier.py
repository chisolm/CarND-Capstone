from styx_msgs.msg import TrafficLight

import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
import glob

# Need to find the right way to add the path to the search path for libs
sys.path.append("..")
sys.path.append("../../../../models/research/")
sys.path.append("../../../../models/research/object_detection/")

from utils import label_map_util
from utils import visualization_utils as vis_util

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image

if tf.__version__ < '1.4.0':
    #print(tf.__version__)
    raise ImportError('Please upgrade your tensorflow installation to v1.4.* or later!')


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        sys.path.append("..")
        sys.path.append("../../../../models/research/")
        sys.path.append("../../../../models/research/object_detection/")
        #print("pwd ", os.getcwd())
        PATH_TO_CKPT = '../../../../CarND-Capstone/models/frozen_models/frozen_ssd_inception_v2_coco.udacity_sim/frozen_inference_graph.pb'

        PATH_TO_LABELS = os.path.join('../../../../CarND-Capstone/data', 'udacity_sim_label_map.pbtxt')
        NUM_CLASSES = 4

        # Load the frozen tensorflow graph into memory
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        # Load label map
        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        category_index = label_map_util.create_category_index(categories)

        pass

    def load_image_into_numpy_array(image):
        (im_width, im_height) = image.size
        return np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        with detection_graph.as_default():
            with tf.Session(graph=detection_graph) as sess:
                # Definite input and output Tensors for detection_graph
                image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
                # Each box represents a part of the image where a particular object was detected.
                detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
                detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = detection_graph.get_tensor_by_name('num_detections:0')
                for image_path in TEST_IMAGE_PATHS:
                    image = Image.open(image_path)
                    # the array based representation of the image will be used later in order to prepare the
                    # result image with boxes and labels on it.
                    image_np = load_image_into_numpy_array(image)
                    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                    image_np_expanded = np.expand_dims(image_np, axis=0)
                    # Actual detection.
                    (boxes, scores, classes, num) = sess.run(
                        [detection_boxes, detection_scores, detection_classes, num_detections],
                        feed_dict={image_tensor: image_np_expanded})

                    # TODO redo test of score/class decode
                    #for ix, score in enumberate(scores):
                    #    if score > .5 and classes[ix] == 2:
                    #        return TrafficLight.RED
        return TrafficLight.UNKNOWN
