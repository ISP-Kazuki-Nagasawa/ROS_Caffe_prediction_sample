#!/usr/bin/env python
#-*- coding: utf-8 -*-

import sys
import time

import numpy as np

### Threading
from threading import Event, Thread
from six.moves.queue import Queue

### OpenCV
import cv2

### ROS
import roslib
roslib.load_manifest('caffe_predictor')

import rospy
from cv_bridge       import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


WINDOW_NAME = "Predictor Window"

bridge = CvBridge()

class Predictor(Thread) :

    def __init__(self, subscriber_name, caffe_root, model_path, deploy_path, gpu_id = -1) :
        super(Predictor, self).__init__()

        self.stop_flag   = Event()
        self.image_queue = Queue()

        ### Load caffe python
        caffe_python_path = "{}/python".format(caffe_root)
        sys.path.insert(0, caffe_python_path)
        caffe = __import__("caffe")

        self.gpu_id = gpu_id
        if gpu_id >= 0 :
            caffe.set_device(gpu_id)
            caffe.set_mode_gpu()
        else :
            caffe.set_mode_cpu()

        ### Load Caffe network
        self.nn = caffe.Net(deploy_path, model_path, caffe.TEST)

        ### Load mean
        mean = None
        mean_binaryproto_path = None
        if mean_binaryproto_path is not None :
            blob = caffe.proto.caffe_pb2.BlobProto()
            blob.ParseFromString(open(mean_binaryproto_path, "rb").read())
            mean = np.array(caffe.io.blobproto_to_array(blob))[0].mean(1).mean(1)

        ### Create transformer
        # 以下、DIGITS 標準の GoogLeNet 学習の場合。
        self.transformer = caffe.io.Transformer({'data': self.nn.blobs['data'].data.shape})
        self.transformer.set_transpose('data', (2, 0, 1))
        if mean is not None :
            self.transformer.set_mean('data', mean)
 
        ### Set NN to batch size of 1
        b, c, self.nn_h, self.nn_w = self.nn.blobs['data'].data.shape
        self.nn.blobs['data'].reshape(1, c, self.nn_h, self.nn_w) 

        ### Start subscriber
        self.sub = rospy.Subscriber(subscriber_name, Image, self.callback)


    def run(self) :


        ### Caffe load (thread内で再読み込みすると高速になる。) 
        import caffe
        if self.gpu_id >= 0 :
            caffe.set_device(self.gpu_id)
            caffe.set_mode_gpu()
        else :
            caffe.set_mode_cpu()


        while True:
            time.sleep(0.01)
            if self.stop_flag.is_set() :
                break

            if self.image_queue.empty() :
                continue

            image = self.image_queue.get()

            ### Prediction
            prediction_start = time.time()
            fractions = np.asarray(self.prediction(image))
            prediction_end = time.time() - prediction_start
            rospy.loginfo("Prediction time : {} sec.".format(prediction_end))

            DISPLAY_TOP_N = 5
            sorted_idxes = np.argsort(fractions)[-1::-1]
            top_n = min(DISPLAY_TOP_N, len(fractions))
            for idx in range(top_n) :
               f_idx = sorted_idxes[idx]
               # text = "{0} : {1} ({2:.4f})".format(idx + 1, labels[f_idx], fractions[f_idx])
               text = "{0} : {1} ({1:.4f})".format(idx + 1, f_idx, fractions[f_idx])
               rospy.loginfo(text)


    def prediction(self, image) :

        ### Resize
        # TODO: ここでは NN Input サイズに合わせる
        resized = cv2.resize(image, (self.nn_w, self.nn_h))

        ### To 3dim tensor
        in_data = self.transformer.preprocess('data', resized)
        in_data = in_data[np.newaxis,]
        
        ### Forward
        self.nn.forward_all(**{self.nn.inputs[0]: in_data})
        output = self.nn.blobs[self.nn.outputs[0]].data[0]
        return output


    def callback(self, msg) :

        try :
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e :
            print(e)
            return

        self.image_queue.put(cv_image)

        cv2.imshow(WINDOW_NAME, cv_image)
        cv2.waitKey(1)


def get_param(node_name, param_name, required = True) :
    param = rospy.get_param('{0}/{1}'.format(node_name, param_name), None)
    if param is None and required is True :
        rospy.logerror("Failed to find parameter %s/%s", node_name, param_name)
    return param


def main() :
    rospy.init_node("predictor.py")

    node_name = rospy.get_name()
    rospy.loginfo("Start node %s", node_name)

    ### Read params (required)
    subscriber_name = get_param(node_name, 'subscriber')
    caffe_root = get_param(node_name, 'caffe_root')
    model_path = get_param(node_name, 'model_path')
    deploy_path = get_param(node_name, 'deploy_path')
    if None in [subscriber_name, caffe_root, model_path, deploy_path] :
        return

    ### Read params (options)
    gpu_id = get_param(node_name, 'gpu_id')
    if gpu_id is None :
        gpu_id = -1
    else :
        gpu_id = int(gpu_id)
    
    ### Predictor
    predictor = Predictor(subscriber_name, caffe_root, model_path, deploy_path, gpu_id)

    predictor.start()
    rospy.spin()

    predictor.stop_flag.set()
    predictor.join()

    rospy.loginfo("Exit node %s", rospy.get_name())


if __name__ == '__main__' :
    main()


