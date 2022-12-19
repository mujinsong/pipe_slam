#!/usr/bin python2
import numpy
import warnings
# import winsound
import numpy as np
from dynamixel_sdk import *
import tf
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from roboticstoolbox import *

lenof_link = 10
a_arm = 0.081
pi=np.pi
print("run")
DHs = [RevoluteDH(a=a_arm, alpha=pi / 2, qlim=[-70 * pi / 180, 70 * pi / 180])]
for i in range(1, lenof_link):
    DHs.append(RevoluteDH(a=a_arm, alpha=(pi / 2) * (-1) ** i, qlim=[-75 * pi / 180, 75 * pi / 180]))
snake = DHRobot(DHs, name="arm")

numpy.set_printoptions(threshold=numpy.inf)

DATA_C = []
DATA_V = []
DATA_P = []
DATA_S = []
DATA_GC = []


# noinspection PyAttributeOutsideInit,PyShadowingBuiltins,GrazieInspection
class dxlControl_X(object):
    """
    Provides group operation support and control for X-series motors in Dynamixel products,
    which can be used for multi-motor cooperative operation.(为Dynamixel产品中的X系电机提供组操作支持和控制，可用于多电机协同作业)

    .. Note::
    Before formally executing the motor control program, please be sure to check the following points to prevent
    the program from failing:
            - When the program is running, please ensure that you are in an absolutely safe area.

            - Please do not forget to turn on the power supply and ensure that the power supply
              module of the unit can work normally.

            - Please check whether all motors in the unit are powered properly.

            - Make sure the communication port is available and connected

    """

    def __init__(self, port_name: str):
        """
        Control Table Address,and related parameters


        :param port_name: port_name: the port name of the Dynamixel
        :type port_name: str

        .. note::
        port naming convention:
                - Windows: "COM*", \n
                - Linux: "/dev/ttyUSB*", \n
                - Mac: "/dev/tty.usbserial-*"

        """
        assert isinstance(port_name, str)
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_LED_ENABLE = 65
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        self.LEN_GOAL_POSITION = 4  # Data Byte Length
        self.LEN_PRESENT_POSITION = 4
        self.DXL_MINIMUM_POSITION_VALUE = 0  # Refer to the Minimum Position Limit of product eManual
        self.DXL_MAXIMUM_POSITION_VALUE = 4095  # Refer to the Maximum Position Limit of product eManual
        self.BAUDRATE = 57600
        self.PROTOCOL_VERSION = 2.0
        self.DEVICENAME = port_name  # 串口
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        self.LED_ENABLE = 1
        self.DXL_MOVING_STATUS_THRESHOLD = 2  # Dynamixel moving status threshold
        self.ADDR_PRESENT_TEMPERATURE = 146  # 当前温度
        self.LEN_PRESENT_TEMPERATURE = 1
        self.ADDR_GOAL_CURRENT = 102  # 目标电流
        self.LEN_GOAL_CURRENT = 2
        self.ADDR_PRESENT_CURRENT = 126  # 当前电流
        self.LEN_PRESENT_CURRET = 2
        self.ADDR_PRESENT_VOLTAGE = 144  # 电压
        self.LEN_PRESENT_VOLTAGE = 2
        self.ADDR_MAX_TEMPERATURE = 31
        self.MAX_TEMPERATURE = 50
        self.ADDR_SPEED_TRAJ = 136
        self.LEN_SPEED_TRAJ = 4

    def __init(self):
        """
        Initialize the port and the communication protocol version
        """
        # Initialize PortHandler instance # Set the port path
        self.portHandler = PortHandler(self.DEVICENAME)
        # Init PacketHandler instance and Set the protocol version
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        # Initialize GroupSyncWrite instance
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_POSITION,
                                             self.LEN_GOAL_POSITION)
        # Initialize GroupSyncRead instace for Present Position
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRESENT_POSITION,
                                           self.LEN_PRESENT_POSITION)

    def open_init_port(self, baud_rate: int):
        """
        Open and initialize the communication port

        :param baud_rate: baud rate of the communication port
        :type baud_rate: int

        """
        self.__init()
        assert isinstance(baud_rate, int)
        self.BAUDRATE = baud_rate
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port!", "Press any key to terminate...", sep='\n')
            sys.exit()
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate", "Press any key to terminate...", sep='\n')
            sys.exit()

    def enable_torque(self, IDs: list):
        """
        Turn on Dynamixl-Xmotor torque

        :param IDs: A list of the ID names of all motors in the group
        :type IDs: list[int]

        Example:
            ``IDs=[0,1,2,3,4,5,6,7,8]``

        .. Note:: IDs should start from zero，and should be continuous

        """
        assert isinstance(IDs, list)
        self.IDs = IDs
        for id in self.IDs:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id,
                                                                           self.ADDR_TORQUE_ENABLE,
                                                                           self.TORQUE_ENABLE)
            # dxl_comm_result_LED, dxl_error_LED = self.packetHandler.write1ByteTxRx(self.portHandler, id,
            #                                                                        self.ADDR_LED_ENABLE,
            #                                                                        self.LED_ENABLE)
            # if dxl_comm_result_LED != COMM_SUCCESS:
            #     pass
            # else:
            #     print('[ID:%03d] Enabled LED' % id)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has been successfully connected" % id)

    
    def move2goal(self, goalPoss):
        rospy.init_node('odom', anonymous=True)
        pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        rate = rospy.Rate(2) # 10hz
        odom_broadcaster = tf.TransformBroadcaster()
        
        """
        :param goalPoss: the Position of every motor
        :type goalPoss: numpy.ndarray

        """
        global DATA_C, DATA_P, DATA_V

        self.enable_torque(self.IDs)
        assert isinstance(goalPoss, numpy.ndarray)
        for id in self.IDs:
            dxl_addparam_result = self.groupSyncRead.addParam(id)
            if not dxl_addparam_result:
                print("[ID:%03d] groupSyncRead add param failed" % id)
                sys.exit()
        for goalPos in goalPoss:
            self.dxl_goal_position = goalPos
            '''--------数据采集-----------'''
            # self.read_Data()
            '''--------------------------'''
            # 将目标位置值分配到字节数组中
            for id in self.IDs:
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(int(self.dxl_goal_position[id]))),
                                       DXL_HIBYTE(DXL_LOWORD(int(self.dxl_goal_position[id]))),
                                       DXL_LOBYTE(DXL_HIWORD(int(self.dxl_goal_position[id]))),
                                       DXL_HIBYTE(DXL_HIWORD(int(self.dxl_goal_position[id])))]
                # 将Dynamixel[id]目标位置添加到Sync-write参数存储
                dxl_addparam_result = self.groupSyncWrite.addParam(id, param_goal_position)
                if not dxl_addparam_result:
                    print("[ID:%03d] groupSyncWrite add param failed" % id)
                    sys.exit()
            # Sync-write goal position
            dxl_comm_result = self.groupSyncWrite.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            # Clear sync-write parameter storage
            self.groupSyncWrite.clearParam()
            while True:
                # Sync-read present position
                dxl_comm_result = self.groupSyncRead.txRxPacket()
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                for id in self.IDs:
                    # Check if groupsyncread data of Dynamixel is available
                    dxl_getdata_result = self.groupSyncRead.isAvailable(id, self.ADDR_PRESENT_POSITION,
                                                                        self.LEN_PRESENT_POSITION)
                    if not dxl_getdata_result:
                        print("[ID:%03d] groupSyncRead getdata failed" % id)
                        sys.exit()
                # 获取 Dynamixel 当前位置值
                for id in self.IDs:
                    dxl_present_position = self.groupSyncRead.getData(id, self.ADDR_PRESENT_POSITION,
                                                                      self.LEN_PRESENT_POSITION)
                    print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t" % (
                        id, self.dxl_goal_position[id], dxl_present_position))
                change_goal_position = 0
                for id in self.IDs:
                    dxl_present_position = self.groupSyncRead.getData(id, self.ADDR_PRESENT_POSITION,
                                                                      self.LEN_PRESENT_POSITION)
                    if not (abs(self.dxl_goal_position[id] - dxl_present_position) > self.DXL_MOVING_STATUS_THRESHOLD):
                        change_goal_position = 1
                        break
                if change_goal_position:
                    break
            # rospy.loginfo("")
            
            qq0 = (goalPos-2048)/4096*2*np.pi
            T = snake.fkine(qq0)
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "/odom"
            current_time = rospy.Time.now()
            odom_quat = tf.transformations.quaternion_from_euler(T.R)
            odom.pose.pose = Pose(T.Trans, Quaternion(*odom_quat))
            pub.publish(odom)
        '''数据采集'''
        # DATA_C = numpy.array(DATA_C)
        # DATA_P = numpy.array(DATA_P)
        # DATA_V = numpy.array(DATA_V)
        # np.save('4-right-angle-notreturn_Present_Current.npy', DATA_C)
        # np.save('4-right-angle-notreturn_Present_Position.npy', DATA_P)
        # np.save('4-right-angle-notreturn_Present_Voltage.npy', DATA_V)
        # np.savetxt('4-right-angle-notreturn_Present_Current.txt', DATA_C)
        # np.savetxt('4-right-angle-notreturn_Present_Position.txt', DATA_P)
        # np.savetxt('4-right-angle-notreturn_Present_Voltage.txt', DATA_V)

        # Clear sync-read parameter storage
        self.groupSyncRead.clearParam()

    def disable_torque(self):
        """
        Turn off the torque of Dynamixl-Xmotor,and close the communication port
        """
        for id in self.IDs:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id,
                                                                           self.ADDR_TORQUE_ENABLE,
                                                                           self.TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        # Close port
        self.portHandler.closePort()

    @staticmethod
    def radarr2Pos(goalPos):
        """弧度转换为电机位置"""
        assert isinstance(goalPos, numpy.ndarray)
        return goalPos / (numpy.pi * 2) * 4096 + 2048

    def present_pos_on_time(self, n):
        """读取当前位置"""
        present_pos = []
        for i in range(n):
            present_pos.append(self.packetHandler.read4ByteTxRx(self.portHandler, i, self.ADDR_PRESENT_POSITION)[0])
        return present_pos

    def Temperature_PreWarning(self, temper_limit: int):
        """温度预警,默认70°"""
        self.tem = temper_limit
        for id in self.IDs:
            present_temperature = self.packetHandler.read1ByteTxRx(self.portHandler, id,
                                                                   self.ADDR_PRESENT_TEMPERATURE)[0]
            if present_temperature >= temper_limit:
                dxl_comm_result_LED, dxl_error_LED = self.packetHandler.write1ByteTxRx(self.portHandler, id,
                                                                                       self.ADDR_LED_ENABLE,
                                                                                       self.LED_ENABLE)
                if dxl_comm_result_LED != COMM_SUCCESS:
                    pass
                else:
                    warnings.warn('[ID:%03d] Enabled LED' % id)
                s = 6
                for i in range(3):
                    warnings.warn('[ID:%03d]号电机工作温度过高(%d°)，%d 秒后将强制关闭力矩' % (id, present_temperature, s))
                    # winsound.Beep(850, 1000)
                    time.sleep(1)
                    s = s - 2
                self.disable_torque()
                quit()

    def read_Data(self):
        """数据采集"""
        global DATA_C, DATA_P, DATA_V
        present_pos = []
        current_value = []
        voltage_value = []
        # current_goal_value = []
        for id in self.IDs:
            dxl_present_position = self.groupSyncRead.getData(id, self.ADDR_PRESENT_POSITION,
                                                              self.LEN_PRESENT_POSITION)
            # 读位置
            present_pos.append(dxl_present_position)
            # 读当前电流
            current_value.append(self.packetHandler.read2ByteTxRx(self.portHandler, id, self.ADDR_PRESENT_CURRENT)[0])
            # 读电压
            voltage_value.append(self.packetHandler.read2ByteTxRx(self.portHandler, id, self.ADDR_PRESENT_VOLTAGE)[0])
            # 读速度轨迹
            # current_goal_value.append(self.packetHandler.read2ByteTxRx(self.portHandler, id, self.ADDR_GOAL_CURRENT)[0])
            print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t" % (
                id, self.dxl_goal_position[id], dxl_present_position))
        for i in range(len(current_value)):
            if current_value[i] >= 60000:
                current_value[i] = ori2dec(current_value[i])
        DATA_P.append(present_pos)
        DATA_C.append(current_value)
        DATA_V.append(voltage_value)
        # DATA_GC.append(current_goal_value)
        # print(f'Present Current: {current_value}')


def pos2rad(arr):
    """将位置信息转换成对应的弧度制"""
    if isinstance(arr, numpy.ndarray):
        return ((arr - 2048) / 2048) * numpy.pi
    else:
        arr = numpy.array(arr)
        return ((arr - 2048) / 2048) * numpy.pi


def ori2dec(ori_num):
    return -(2 ** 16 - ori_num)


if __name__ == '__main__':
    d = dxlControl_X('COM3')
    d.open_init_port(3000000)
    d.enable_torque([0, 1, 2, 3, 4, 5])
    d.move2goal(numpy.array([2048, 2048, 2048, 2048, 2048, 2048]))
    d.disable_torque()
