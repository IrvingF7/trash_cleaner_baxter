import cv2
import numpy as np

def find_trash(model_output, confid_thresh, c_img, d_img):
	bboxes = model_output['detection_boxes']
	classes = model_output['detection_classes']
	scores = model_output['detection_scores']

	valid_indxs = sum([i > confid_thresh for i in scores])
	valid_trash = []

	for i in range(valid_indxs):
		points = bboxes[i]
		points = _scale_bbox2img(self, points, c_img)
		label_num = classes[i]

		trash = Trash(points, label_num, c_img, d_img)
		valid_trash.append(trash)

	return valid_trash

def _scale_bbox2img(self, points, c_img):
	height, width, dim = c_img.shape

	x_min = points[0] * width
	y_min = points[1] * height
	x_max = points[2] * width
	y_max = points[3] * height

	return [x_min, y_min, x_max, y_max]

class Trash(object):

	def __init__(self, model_output, label_num, c_img, d_img):
		DEPTH_REGION = 20

		self.c_img = c_img
		self.d_img = d_img

		self.points = dict()
		self.points['x_min'] = points[0]
		self.points['x_max'] = points[2]
		self.points['y_min'] = points[1]
		self.points['y_max'] = points[3]

		self.cg = self._find_cg()
		self.depth = self._find_depth(DEPTH_REGION)

		self.label_num = label_num

	def get_cg(self):
		return self.cg

	def get_depth(self):
		return self.depth

	def get_label_num(self):
		return self.label_num

	def _find_cg(self):
		cg_x = int((self.points['x_min'] + self.points['x_max']) / 2)
		cg_y = int((self.points['y_min'] + self.points['y_max']) / 2)
		cg = (cg_x, cg_y)
		return cg

	def _find_depth(self, depth_region):
		x = self.cg[0]
		y = self.cg[1]

		depths = d_img[y-depth_region:y+depth_region, x-depth_region:x+depth_region]
		mean_depth = np.mean(depths)

		return mean_depth