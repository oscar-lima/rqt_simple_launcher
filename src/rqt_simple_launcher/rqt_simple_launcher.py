#!/usr/bin/env python3

import os
import sys
import signal
import time

import rospy
import rospkg
import yaml

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFileDialog
from std_msgs.msg import String

from PyQt5.QtWidgets import QApplication

import subprocess

class OpenFileDialog(QWidget):
    '''
    this will open a dialog to select and open a yaml file
    '''
    def __init__(self, initial_path=None):
        super().__init__()
        left, top, width, height = 10, 10, 640, 480
        self.setGeometry(left, top, width, height)
        self.initial_path = initial_path

    def openFileNameDialog(self, save_file_name_dialog=False):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        if self.initial_path is None:
            initial_path = os.environ['HOME']
        else:
            initial_path = self.initial_path
        if save_file_name_dialog:
            fileName, _ = QFileDialog.getSaveFileName(self, 'Select boxes yaml file',\
                          initial_path, 'Yaml Files (*.yaml)', options=options)
        else:
            fileName, _ = QFileDialog.getOpenFileName(self, 'Select boxes yaml file',\
                          initial_path, 'Yaml Files (*.yaml)', options=options)
        if fileName:
            return fileName

    def saveFileNameDialog(self):
        return self.openFileNameDialog(save_file_name_dialog=True)

class RqtSimpleLauncher(Plugin):

    def __init__(self, context):
        super(RqtSimpleLauncher, self).__init__(context)
        rospy.loginfo('Initializing rqt_simple_launcher')

        self.setObjectName('RqtSimpleLauncher')
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file (xml description of the gui window created with qtcreator)
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_simple_launcher'), 'config', 'rqt_simple_launcher.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('rqt_simple_launcher.ui')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # used when running multiple instances of this plugin as panels within rqt_gui
        middle_ns = rospy.get_param(f'~args_config_{context.serial_number()}', None)
        if middle_ns is None:
            middle_ns = ''
        else:
            middle_ns = middle_ns + '/'

        # class variables
        self.exec_on = False
        self.exec_process = None

        # parameters
        # False: really launches the launch file, True: only prints the command that will be launched for debug purposes
        self.only_print_cmd = rospy.get_param(f'~{middle_ns}only_print_cmd', False)
        brief = rospy.get_param(f'~{middle_ns}brief', 'Bringup sim')
        self.is_launch_file = True
        self.launch = rospy.get_param(f'~{middle_ns}launch', {})
        if self.launch == {}:
            self.is_launch_file = False
        self.is_ros_node = True
        self.run = rospy.get_param(f'~{middle_ns}run', {})
        if self.run == {}:
            self.is_ros_node = False
        else:
            self._widget.chkBoolParam.hide()
        # sanity check, either run or launch param must be set
        if self.launch == {} and self.run == {}:
            rospy.logerr('params not correctly configured (both launch and run params are missing), will exit')
            sys.exit(0)
        # sanity check, can't be launch file and ros node at the same time
        assert(self.is_launch_file ^ self.is_ros_node)  # XOR
        self.bool_arg = rospy.get_param(f'~{middle_ns}bool_arg', {})
        self.args_dictionary = rospy.get_param(f'~{middle_ns}args', {})
        required_services = rospy.get_param(f'~{middle_ns}required_services', [])
        self.robot_namespace = rospy.get_param(f'~{middle_ns}robot_namespace', 'mobipick')
        placeholder_values = rospy.get_param(f'~{middle_ns}placeholder_values', {'robot_namespace': self.robot_namespace})
        self.required_services = self.replace_placeholders(required_services, placeholder_values)
        self.execution_timeout = rospy.get_param(f'~{middle_ns}execution_timeout', 60)

        # construct launch file path
        if self.is_launch_file:
            if self.launch == {}:
                rospy.logerr('Required parameter launch not set, exiting')
                sys.exit(0)
            else:
                pkg_path = rospkg.RosPack().get_path(self.launch['ros_pkg'])
                folder = self.launch['folder']
                launch_file = self.launch['launch_file']
                self.launch_file_path = f'{pkg_path}/{folder}/{launch_file}'

        # set label of what the button launch will do
        self._widget.groupButtons.setTitle(brief)

        # handle bool param, if required set its text, otherwise hide the widget
        if self.bool_arg == {}:
            self._widget.chkBoolParam.hide()
        else:
            self._widget.chkBoolParam.setText(self.bool_arg['text'])
            self._widget.chkBoolParam.setChecked(self.bool_arg['initial_state'])

        # hide all combo boxes with it's labels
        self.comboArgList = [self._widget.comboArg1, self._widget.comboArg2, self._widget.comboArg3, self._widget.comboArg4]
        for comboArg in self.comboArgList:
            comboArg.hide()
        self.lblArgList = [self._widget.lblArg1, self._widget.lblArg2, self._widget.lblArg3, self._widget.lblArg4]
        for lblArg in self.lblArgList:
            lblArg.hide()

        # show only the required combo boxes with it's labels
        for i, arg in enumerate(self.args_dictionary):
            # show combo box
            self.lblArgList[i].show()
            self.lblArgList[i].setText(arg['text'])
            # fill combo box with options
            self.comboArgList[i].show()
            self.comboArgList[i].addItems(arg['options'])

        # set default option for each combo box
        for i, arg in enumerate(self.args_dictionary):
            index = self.comboArgList[i].findText(arg['selected_option'])
            self.comboArgList[i].setCurrentIndex(index)

        # make button orange at startup
        self.configure_exec_button_down()

        self._widget.groupArguments.setEnabled(True)

        ## make a connection between the qt objects and this class methods
        self._widget.cmdLaunch.clicked.connect(self.cmd_exec_clicked)
        self._widget.cmdSaveConfig.clicked.connect(self.cmd_save_config_clicked)

        # to trigger the execution of a sibling process (other instances of rqt_simple_launcher) via topic
        self.trigger_pub = rospy.Publisher(f'~{middle_ns}exec_sibling_process', String, queue_size=1)
        # external trigger of executing a process via topic
        rospy.Subscriber(f'~{middle_ns}execute_process', String, self.execute_process)

        context.add_widget(self._widget)

        # to catch Ctrl + C signal from keyboard and close stream properly
        signal.signal(signal.SIGINT, self.signal_handler)

        rospy.loginfo(f'{brief} initialized')
        # end of constructor

    # ::::::::::::::  class methods

    def shutdown_plugin(self):
        rospy.loginfo('I detected you want to quit, calling destructor')
        self.__del__()

    def signal_handler(self, sig, frame):
        rospy.loginfo('You pressed Ctrl+C, calling destructor')
        self.__del__()

    def __del__(self):
        '''
        Destructor: perform cleanup
        '''
        self.kill_exec_process()
        rospy.loginfo('bye bye!')
        sys.exit(0)

    def execute_process(self, msg):
        '''
        Callback function to execute a process via topic
        '''
        rospy.logwarn('received trigger to exec process via topic')
        self.cmd_exec_clicked()

    def configure_exec_button_down(self):
        button_common_text = f'Down\nclick to\n'
        if self.is_launch_file:
            button_text = f'{button_common_text}Launch'
        elif self.is_ros_node:
            button_text = f'{button_common_text}Run'
        else:
            button_text = 'error'
        self.configure_exec_button('orange', button_text)

    def loginfo_green(self, message):
        '''
        This function acts as a wrapper around rospy.loginfo to print messages in green color.
        '''
        green_start = '\033[92m'
        color_reset = '\033[0m'
        rospy.loginfo(green_start + str(message) + color_reset)

    def replace_placeholders(self, item, placeholder_values):
        if isinstance(item, dict):
            for key, value in item.items():
                item[key] = self.replace_placeholders(value, placeholder_values)
        elif isinstance(item, list):
            return [self.replace_placeholders(elem, placeholder_values) for elem in item]
        elif isinstance(item, str):
            return item.format(**placeholder_values)
        return item

    def cmd_exec_clicked(self):
        self.exec_on = not self.exec_on
        if self.exec_on:
            if self.is_launch_file:
                self.launch_file_process()
            elif self.is_ros_node:
                self.run_node()
            else:
                rospy.logerr('malformed yaml config file (either launch or run param is missing), will exit')
                sys.exit(0)
        else:
            self.kill_exec_process()

    def configure_exec_button(self, color, text):
        self._widget.cmdLaunch.setText(f'State: {text}')
        self._widget.cmdLaunch.setStyleSheet(f'background-color: {color}; color: white;')
        QApplication.processEvents()

    def make_launch_args(self):
        cmd_args = []
        cmd_args.append(f"{self.bool_arg['arg']}:={self._widget.chkBoolParam.isChecked()}")
        cmd_args.extend([f"{self.args_dictionary[i]['arg']}:={comboArg.currentText()}"
                         for i, comboArg in enumerate(self.comboArgList) if comboArg.isVisible()])
        return cmd_args

    def make_run_args(self):
        # TODO: not sure what to do with boolean arg for ros nodes...
        # bool_arg_value = False
        # if self._widget.chkBoolParam.isChecked():
            # bool_arg_value = True
        # cmd_args.append(f"{self.bool_arg['arg']}:={bool_arg_value}")
        return [comboArg.currentText() for comboArg in self.comboArgList if comboArg.isVisible()]

    def check_cmd(self, cmd):
        # prevent shell injection
        forbidden_chars = [' ', '|', '&', ';', '`', '$', '(', ')', '{', '}', '[', ']',
                           '*', '?', '~', '!', '<', '>', '\n', '\r', '\\', '"', "'"]
        if any(char in cmd for char in forbidden_chars):
            rospy.logerr(f'You can not use {forbidden_chars} in your command, will exit')
            sys.exit(0)

    def service_exists(self, service_name):
        try:
            output = subprocess.check_output(['rosservice', 'list'])
            return service_name in output.decode('utf-8').split('\n')
        except subprocess.CalledProcessError:
            return False

    def wait_for_services(self, required_services, timeout=60):
        '''
        Function to wait for required services to be available
        if timeout occurs then kills the process
        also handles enable / disable of the arguments group and sets appropriate button text and color
        '''
        if self.only_print_cmd:
            rospy.loginfo('Not waiting for services as only_print_cmd is True')
            return True
        start_time = time.time()
        while time.time() - start_time < timeout:
            if all(self.service_exists(service) for service in required_services):
                rospy.loginfo('All services are available!')
                self.configure_exec_button('green', 'Running\nclick to\ntake down')
                self._widget.groupArguments.setEnabled(False)
                return
            QApplication.processEvents() # prevent the system thinking that the application is not responding
            time.sleep(1)
        rospy.logerr('==============================================================')
        rospy.logerr(f'\nTimeout: Required services were unavailable after {timeout} s. Killing process :\n')
        rospy.logerr(f'{self.launch_file_path}')
        rospy.logerr('==============================================================')
        self.kill_exec_process()
        self._widget.groupArguments.setEnabled(True)

    def pretty_print_cmd(self, cmd):
        return ' '.join(cmd)

    def launch_file_process(self):
        self.configure_exec_button('red', 'Launching\nplease wait...')
        self._widget.groupArguments.setEnabled(False)
        self.check_cmd(self.launch_file_path)
        cmd_args = self.make_launch_args()
        for arg in cmd_args:
            self.check_cmd(arg)
        cmd = ['roslaunch', self.launch_file_path] + ['__ns:=/'] + cmd_args
        rospy.loginfo(f'Launching:')
        self.loginfo_green(self.pretty_print_cmd(cmd))
        if not self.only_print_cmd:
            # shell=False is important as it is safer
            self.exec_process = subprocess.Popen(cmd, shell=False, preexec_fn=os.setsid)
        self.wait_for_services(self.required_services, timeout=self.execution_timeout)
        self.trigger_pub.publish(String('done'))

    def wait_for_node_to_finish(self, process, timeout=30):
        '''
        we could use process.wait() but it is blocking and we don't want rqt msgs saying process is not responding
        this method esentially does the same as process.wait() but in a non-blocking way and with a timeout
        unlike the wait for services approach used for launch files, here we don't kill the process if timeout occurs
        but after the timeout we enable the arguments group and allow the user to bring down the node manually by pressing the toggle button
        '''
        if self.only_print_cmd:
            rospy.loginfo('Not waiting for node to finish as only_print_cmd is True')
            return True
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.is_node_finished(process):
                rospy.loginfo('Node is finished with execution!')
                self.configure_exec_button_down()
                return
            QApplication.processEvents() # prevent the system thinking that the application is not responding
            time.sleep(1)
        self._widget.groupArguments.setEnabled(True)
        self.exec_on = False

    def is_node_finished(self, process):
        # Check if the process has finished
        if process.poll() is None:
            # Process is still running
            return False
        else:
            # Process finished
            # exit_code = process.poll()
            return True

    def run_node(self):
        self.configure_exec_button('red', 'Running\nplease wait...')
        self._widget.groupArguments.setEnabled(False)
        ros_pkg = self.run['ros_pkg']
        self.check_cmd(ros_pkg)
        cmd_args = self.make_run_args()
        for arg in cmd_args:
            self.check_cmd(arg)
        cmd = ['rosrun', ros_pkg, self.run['ros_node']] + ['__ns:=/'] + cmd_args
        rospy.loginfo(f'Running:')
        self.loginfo_green(self.pretty_print_cmd(cmd))
        if not self.only_print_cmd:
            # shell=False is important as it is safer
            self.exec_process = subprocess.Popen(cmd, shell=False, preexec_fn=os.setsid)
        self.wait_for_node_to_finish(self.exec_process, timeout=self.execution_timeout)
        self._widget.groupArguments.setEnabled(False)

    def kill_exec_process(self):
        '''
        Function to send SIGINT to the launch file's process
        '''
        self.configure_exec_button('red', 'Killing...\nplease wait...')
        self._widget.groupArguments.setEnabled(False)
        if self.exec_process:
            if not self.only_print_cmd:
                try:
                    os.killpg(os.getpgid(self.exec_process.pid), signal.SIGINT)
                except ProcessLookupError:
                    rospy.logwarn('Process already killed')
                # wait for the process to complete
                self.exec_process.wait()
            self.exec_process = None
        self.configure_exec_button_down()
        self._widget.groupArguments.setEnabled(True)

    def cmd_save_config_clicked(self):
        rospy.loginfo('cmd_save_config_clicked')
        sfd = OpenFileDialog()
        yaml_path = sfd.saveFileNameDialog()
        if not '.yaml' in yaml_path:
            yaml_path = yaml_path + '.yaml'
        # create master dictionary
        master_dict = {}
        master_dict.update({'brief': self._widget.groupButtons.title()})
        master_dict.update({'launch': self.launch})
        bool_arg_checked = False
        if self._widget.chkBoolParam.isChecked():
            bool_arg_checked = True
        self.bool_arg.update({'initial_state': bool_arg_checked})
        master_dict.update({'bool_arg': self.bool_arg})
        master_dict.update({'args': self.args_dictionary})
        # get selected combo box options
        for i, arg in enumerate(self.args_dictionary):
            master_dict['args'][i].update({'selected_option': self.comboArgList[i].currentText()})
        rospy.loginfo(f'writing current configuration to yaml file: {yaml_path}')
        with open(yaml_path, 'w') as yaml_file:
            yaml.dump(master_dict, yaml_file, default_flow_style=False)
