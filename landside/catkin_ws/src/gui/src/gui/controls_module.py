from __future__ import print_function

from std_srvs.srv import SetBool

import os
import time
import sys

import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QFileDialog, QGraphicsView, QIcon, QWidget

import paramiko.client.SSHClient

#TODO make checkbox for sim mode

class ControlsWidget(QWidget):
    """
    Widget for use with Bag class to display and replay bag files
    Handles all widget callbacks and contains the instance of BagTimeline for storing visualizing bag data
    """
    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(ControlsWidget, self).__init__()

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print ('arguments: ', args)
            print ('unknowns: ', unknowns)

        rp = rospkg.RosPack() # delete later
        ui_file = os.path.join(rospkg.RosPack().get_path('gui'), 'resource', 'ControlsInterface.ui')
        loadUi(ui_file, self)

        self.setObjectName('ControlsWidget')

        self.graphics_view.resizeEvent = self._resizeEvent
        self.graphics_view.setMouseTracking(True)

        # define buttons
        # self.pose_enable = QIcon.fromTheme() # fill in with icon
        # self.twist_enable = QIcon.fromTheme()
        # self.run_launch_file = QIcon.fromTheme()
        # self.controls_enable = QIcon.fromTheme()

        self.pose_enable.clicked[bool].connect(self._handle_pose_enable_clicked)
        self.twist_enable.clicked[bool].connect(self._handle_twist_enable_clicked)
        self.run_launch_file.clicked[bool].connect(self._handle_run_launch_file_clicked)
        self.controls_enable.clicked[bool].connect(self._handle_controls_enable_clicked)

        # example: self.play_icon = QIcon.fromTheme('media-playback-start')

        self.x_pose.singleStep(0.1)
        self.y_pose.singleStep(0.1)
        self.z_pose.singleStep(0.1)
        self.roll_pose.singleStep(0.1)
        self.pitch_pose.singleStep(0.1)
        self.yaw_pose.singleStep(0.1)
        self.x_twist.singleStep(0.1)
        self.y_twist.singleStep(0.1)
        self.z_twist.singleStep(0.1)
        self.roll_twist.singleStep(0.1)
        self.pitch_twist.singleStep(0.1)
        self.yaw_twist.singleStep(0.1)

        self.closeEvent = self.handle_close
        self.keyPressEvent = self.on_key_press
        # TODO when the closeEvent is properly called by ROS_GUI implement that event instead of destroyed
        self.destroyed.connect(self.handle_destroy)

        self.graphics_view.keyPressEvent = self.graphics_view_on_key_press
        self.play_button.setEnabled(False)
        self.thumbs_button.setEnabled(False)
        self.zoom_in_button.setEnabled(False)
        self.zoom_out_button.setEnabled(False)
        self.zoom_all_button.setEnabled(False)
        self.faster_button.setEnabled(False)
        self.slower_button.setEnabled(False)
        self.begin_button.setEnabled(False)
        self.end_button.setEnabled(False)
        self.save_button.setEnabled(False)

        self._recording = False

        self._timeline.status_bar_changed_signal.connect(self._update_status_bar)

        # Initializing SSH clients for all available commands
        self.launch_file_client = SSHClient()
        self.launch_file_client.load_system_host_keys()

        self.publish_pose_client = SSHClient()
        self.publish_pose_client.load_system_host_keys()

        self.publish_twist_client = SSHClient()
        self.publish_twist_client.load_system_host_keys()



    def graphics_view_on_key_press(self, event):
        key = event.key()
        if key in (Qt.Key_Left, Qt.Key_Right, Qt.Key_Up, Qt.Key_Down, Qt.Key_PageUp, Qt.Key_PageDown):
            # This causes the graphics view to ignore these keys so they can be caught by the bag_widget keyPressEvent
            event.ignore()
        else:
            # Maintains functionality for all other keys QGraphicsView implements
            QGraphicsView.keyPressEvent(self.graphics_view, event)

    # callbacks for ui events
    def on_key_press(self, event):
        key = event.key()
        if key == Qt.Key_Space:
            self._timeline.toggle_play()
        elif key == Qt.Key_Home:
            self._timeline.navigate_start()
        elif key == Qt.Key_End:
            self._handle_end_clicked()
        elif key == Qt.Key_Plus or key == Qt.Key_Equal:
            self._handle_faster_clicked()
        elif key == Qt.Key_Minus:
            self._handle_slower_clicked()
        elif key == Qt.Key_Left:
            self._timeline.translate_timeline_left()
        elif key == Qt.Key_Right:
            self._timeline.translate_timeline_right()
        elif key == Qt.Key_Up or key == Qt.Key_PageUp:
            self._handle_zoom_in_clicked()
        elif key == Qt.Key_Down or key == Qt.Key_PageDown:
            self._handle_zoom_out_clicked()

    def handle_destroy(self, args):
        self._timeline.handle_close()

    def handle_close(self, event):
        self.shutdown_all()

        event.accept()

    def _resizeEvent(self, event):
        # TODO The -2 allows a buffer zone to make sure the scroll bars do not appear when not needed. On some systems (Lucid) this doesn't function properly
        # need to look at a method to determine the maximum size of the scene that will maintain a proper no scrollbar fit in the view.
        self.graphics_view.scene().setSceneRect(0, 0, self.graphics_view.width() - 2, max(self.graphics_view.height() - 2, self._timeline._timeline_frame._history_bottom))

    # our methods
    def _handle_pose_enable_clicked(self):
        linear_x  = self.x_pose.value()
        linear_y  = self.y_pose.value()
        linear_z  = self.z_pose.value()
        angular_x = self.roll_pose.value()
        angular_y = self.pitch_pose.value()
        angular_z = self.yaw_pose.value()
        # TODO create from text input 
        

        command_format = ("rostopic pub -r 15 /controls/desired_pose geometry_msgs/Pose " +
        "'{{linear: {{x: {lin_x}, y: {lin_y}, z: {lin_z}}}, angular: {{x: {ang_x}, y: {ang_y}, z: {ang_z}}} }}'").format(lin_x = linear_x, lin_y=linear_y, lin_z=linear_z, 
        ang_x=angular_x, ang_y=angular_y, ang_z=angular_z)

        self.publish_pose_client.connect('192.168.1.1',port=2200,username='root',password='robotics')
        stdin, stdout, stderr = self.publish_pose_client.exec_command(command_format)
        self.publish_pose_client.close()
    
    def _handle_twist_enable_clicked(self):
        linear_x  = self.x_twist.value()
        linear_y  = self.y_twist.value()
        linear_z  = self.z_twist.value()
        angular_x = self.roll_twist.value()
        angular_y = self.pitch_twist.value()
        angular_z = self.yaw_twist.value()
        # TODO create from text input


        command_format = ("rostopic pub -r 15 /controls/desired_twist geometry_msgs/Twist " +
        "'{{linear: {{x: {lin_x}, y: {lin_y}, z: {lin_z}}}, angular: {{x: {ang_x}, y: {ang_y}, z: {ang_z}}} }}'").format(lin_x = linear_x, lin_y=linear_y, lin_z=linear_z, 
        ang_x=angular_x, ang_y=angular_y, ang_z=angular_z)

        self.publish_twist_client.connect('192.168.1.1',port=2200,username='root',password='robotics')
        stdin, stdout, stderr = self.publish_twist_client.exec_command(command_format)
        self.publish_twist_client.close()

    def _handle_run_launch_file_clicked(self): 
        # TODO Check with Muthu

        self.launch_file_client.connect('192.168.1.1',port=2200,username='root',password='robotics')
        stdin, stdout, stderr = self.launch_file_client.exec_command('roslaunch execute motion.launch')
        self.launch_file_client.close()

    def _handle_controls_enable_clicked(self):
        rospy.wait_for_service('enable_controls')
        try:
            enable_controls = rospy.ServiceProxy('enable_controls', SetBool)
            res = enable_controls(True) # TODO gui will need to say something about failure for res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e) 
        
    # preexisting methods that aren't necessary 
    def _handle_load_clicked(self):
        filename = QFileDialog.getOpenFileName(self, self.tr('Load from File'), '.', self.tr('Bag files {.bag} (*.bag)'))
        if filename[0] != '':
            self.load_bag(filename[0])

    def load_bag(self, filename):
        bag = rosbag.Bag(filename)
        self.play_button.setEnabled(True)
        self.thumbs_button.setEnabled(True)
        self.zoom_in_button.setEnabled(True)
        self.zoom_out_button.setEnabled(True)
        self.zoom_all_button.setEnabled(True)
        self.faster_button.setEnabled(True)
        self.slower_button.setEnabled(True)
        self.begin_button.setEnabled(True)
        self.end_button.setEnabled(True)
        self.save_button.setEnabled(True)
        self.record_button.setEnabled(False)
        self._timeline.add_bag(bag)

    def _handle_save_clicked(self):
        filename = QFileDialog.getSaveFileName(self, self.tr('Save selected region to file...'), '.', self.tr('Bag files {.bag} (*.bag)'))
        if filename[0] != '':
            self._timeline.copy_region_to_bag(filename[0])

    # Shutdown all members

    def shutdown_all(self):
        self._timeline.handle_close()