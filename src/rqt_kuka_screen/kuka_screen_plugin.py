#!/usr/bin/env python

from qt_gui.plugin import Plugin
from std_msgs.msg import String
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtWidgets import *
import rospy, cv2
import numpy as np

class KukaScreenPlugin(Plugin):
	def __init__(self, context):
		super(KukaScreenPlugin, self).__init__(context)
		self.context = context
		self.mode = 'learn'
		self._widget = KukaScreenWidget()
		self.context.add_widget(self._widget)
		self.sub_mode = rospy.Subscriber('/command_init', String, self.callback, queue_size = 10)

	def callback(self, data):
		c = data.data.lower()
		if c != self.mode:
			self.mode = c
			self._widget.update_mode(self.mode)

class KukaScreenWidget(QWidget):
	def __init__(self):
		super(KukaScreenWidget, self).__init__()
		# layout variables
		self.main_layout = QVBoxLayout()
		self.l_progress_1 = QProgressBar()
		self.l_progress_2 = QProgressBar()
		self.l_video_1 = QLabel()
		self.l_video_2 = QLabel()
		self.l_video_3 = QLabel()
		self.d_video_1 = QLabel()
		self.d_video_2 = QLabel()
		self.d_video_3 = QLabel()
		self.t_video_1 = QLabel()
		# global variables
		self.bridge = CvBridge()
		self.window_size = self.size()
		self.pix_width = (self.window_size.width() / 3) - 30
		self.mode = 'learn'
		self.initialize_widgets()
		self.setLayout(self.main_layout)
		##### testing subscribers #####
		self.s1 = rospy.Subscriber('/cam1/image_raw/compressed', CompressedImage, self.cb_learn1, queue_size = 10)
		self.s2 = rospy.Subscriber('/cam2/image_raw/', Image, self.cb_learn2, queue_size = 10)
		self.s6 = rospy.Subscriber('/cam2/image_raw/compressed', CompressedImage, self.cb_detect1, queue_size = 10)
		self.s7 = rospy.Subscriber('/cam1/image_raw/', Image, self.cb_detect2, queue_size = 10)
		self.s8 = rospy.Subscriber('/cam2/image_raw/', Image, self.cb_detect3, queue_size = 10)
		##### Actual subscriber topics #####
		# self.s1 = rospy.Subscriber(''/cam1/camera/image_raw/compressed', CompressedImage, self.cb_learn1, queue_size = 10)
		# self.s2 = rospy.Subscriber('/crop', Image, self.cb_learn2, queue_size = 10))
		# self.s3 = rospy.Subscriber('/map', Image, self.cb_learn3, queue_size = 10)
		self.s4 = rospy.Subscriber('/learning_progress', String, self.cb_progress1, queue_size = 10)
		self.s5 = rospy.Subscriber('/feature_progress', String, self.cb_progress2, queue_size = 10)
		# self.s6 = rospy.Subscriber('/kinect2/hd/image_color_rect/compressed', CompressedImage, self.cb_detect1, queue_size = 10)
		# self.s7 = rospy.Subscriber('/cl_image', Image, self.cb_detect2,  queue_size = 10)
		# self.s8 = rospy.Subscriber('/imgnet_image', Image, self.cb_detect3,  queue_size = 10)
		# self.s9 = rospy.Subscriber('/camera/rgb/image_rect_color/compressed', CompressedImage, self.cb_task, queue_size = 10)

	def initialize_widgets(self):
		self.stacked_widget = QStackedWidget()
		self.learn_widget = QWidget()
		self.detect_widget = QWidget()
		self.task_widget = QWidget()

		self.learning_layout = self.learn_layout()
		self.detecting_layout = self.detect_layout()
		self.tasking_layout = self.task_layout()

		self.learn_widget.setLayout(self.learning_layout)
		self.detect_widget.setLayout(self.detecting_layout)
		self.task_widget.setLayout(self.tasking_layout)

		self.stacked_widget.addWidget(self.learn_widget)
		self.stacked_widget.addWidget(self.detect_widget)
		self.stacked_widget.addWidget(self.task_widget)
		self.main_layout.addWidget(self.stacked_widget)

	def learn_layout(self):
		self.setWindowTitle('Learning')
		l_layout = QVBoxLayout()
		l_imgs = QHBoxLayout()
		self.l_progress_1.setRange(0, 100)
		self.l_progress_2.setRange(0, 100)
		l_imgs.addWidget(self.l_video_1)
		l_imgs.addWidget(self.l_video_2)
		l_imgs.addWidget(self.l_video_3)
		l_layout.addLayout(l_imgs)
		l_layout.addWidget(self.l_progress_1)
		l_layout.addWidget(self.l_progress_2)
		return l_layout

	def detect_layout(self): 
		self.setWindowTitle('Detecting')
		d_layout = QHBoxLayout()
		d_layout.addWidget(self.d_video_1)
		d_layout.addWidget(self.d_video_2)
		d_layout.addWidget(self.d_video_3)
		return d_layout

	def task_layout(self):
		self.setWindowTitle('Task Completion')
		t_layout = QVBoxLayout()
		t_layout.addWidget(self.t_video_1)
		return t_layout

	def convert_img(self, data):
		try:
			frame = self.bridge.imgmsg_to_cv2(data, "rgb8")
		except CvBridgeError as e:
			print e
		image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
		pixmap = QPixmap.fromImage(image)
		pixmap = pixmap.scaledToWidth(self.pix_width)
		return pixmap

	def convert_compressed_img(self, data):
		np_arr = np.fromstring(data.data, np.uint8)
		frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
		pixmap = QPixmap.fromImage(image)
		pixmap = pixmap.scaledToWidth(self.pix_width)
		return pixmap 

	def cb_progress1(self, data):
		self.l_progress_1.setValue(int(data.data))

	def cb_progress2(self, data):
		self.l_progress_2.setValue(int(data.data)) 

	def cb_learn1(self, data):
		if self.mode == 'learn':
			pixmap = self.convert_compressed_img(data)
			self.l_video_1.setPixmap(pixmap)

	def cb_learn2(self, data):
		if self.mode == 'learn':
			pixmap = self.convert_img(data)
			self.l_video_2.setPixmap(pixmap)

	def cb_learn3(self, data):
		if self.mode == 'learn':
			pixmap = self.convert_img(data)
			self.l_video_3.setPixmap(pixmap)

	def cb_detect1(self, data):
		if self.mode == 'detect':
			pixmap = self.convert_compressed_img(data)
			self.d_video_1.setPixmap(pixmap)

	def cb_detect2(self, data):
		if self.mode == 'detect':
			pixmap = self.convert_img(data)
			self.d_video_2.setPixmap(pixmap)

	def cb_detect3(self, data):
		if self.mode == 'detect':
			pixmap = self.convert_img(data)
			self.d_video_3.setPixmap(pixmap)

	def cb_task(self, data):
		if self.mode == 'task':
			pixmap = self.convert_img(data)
			self.t_video_1.setPixmap(pixmap)

	def activate_widget(self, index):
		i = self.stacked_widget.currentIndex()
		if i != index:
			self.stacked_widget.setCurrentIndex(index)
			self.window_size = self.size()
		else:
			print 'activating error'

	def change_widget(self):
		if self.mode == 'learn':
			self.activate_widget(0)
			self.pix_width = (self.window_size.width() / 3) - 30
		elif self.mode == 'detect':
			self.activate_widget(1)
			self.pix_width = (self.window_size.width() / 3) - 30
		elif self.mode == 'task':
			self.activate_widget(2)
			self.pix_width = self.window_size.width() - 30
		else:
			print 'change widget error'

	def update_mode(self, mode):
		self.mode = mode
		print 'CHANGED TO: ', self.mode
		self.change_widget()