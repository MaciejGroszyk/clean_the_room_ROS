import roslib
import rospy
import smach
import smach_ros
import json
import tf

from std_msgs.msg import String, Float32MultiArray, Bool
from geometry_msgs.msg import TransformStamped

dock_location= []
DATA_JSON_FILE_LOC='../dock_param/data.json'
DATA_TXT_FILE_LOC='../dock_param/data.txt'

pub_room_name = None
pub_obstacle_loc = None


def publish_room_name(room_name):
    iterations = 0
    while(iterations<100):
        rospy.loginfo(room_name)
        pub_room_name.publish(room_name)
        iterations += 1
        rospy.sleep(0.1)

# define state Init
class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dock_exists', 'dock_doesnt_exist'])

    def execute(self, userdata):
        rospy.loginfo(DATA_JSON_FILE_LOC)

        # zczytanie danych stacji dokujacej /home/mateusz/ima_ws/src/ima_22l_lab6_groszyk_zembron/package_groszyk_zembron/dock_param/data.json
        f = open('/home/mateusz/ima_ws/src/ima_22l_lab6_groszyk_zembron/package_groszyk_zembron/dock_param/data.json')
        # f = open('../dock_param/data.json')
        data = json.loads(f.read())
        dock_location= data['location']
        f.close()

        if (dock_location[0] is not None and dock_location[1] is not None):
            return 'dock_exists'
        else:
            return 'dock_doesnt_exist'

# define state FindDock
class FindDock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dock_exists'])
        self.dock_found= False
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        while (not self.dock_found):
            try:
                (trans,rot) = self.tf_listener.lookupTransform('/odom', '/symbol_power', rospy.Time(0))
                print(trans)
                dock_location= trans;     
                if (dock_location[0] is not None and dock_location[1] is not None): 
                    dictionary={'location': dock_location}
                    with open(DATA_JSON_FILE_LOC, "w") as outfile:
                        json.dump(dictionary, outfile)
                    with open(DATA_TXT_FILE_LOC, 'w') as f:
                        f.write( str(dock_location[0]) +  " " + str(dock_location[1]))

                    return 'dock_exists'         

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            #rospy.loginfo('Dock location not found')


# define state Cleaning
class Cleaning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['obstacle_found','end_of_cleaning'])
        self.tf_listener1 = tf.TransformListener()
        self.obstacle_found = False
        self.end_of_cleaning = False
        rospy.Subscriber("/RoomCleanerState", String, self.callback, queue_size=1)

        rospy.loginfo('Executing state FIND_DOCK')
        room_name = "lazienka" #sprzatany pokoj
        rospy.loginfo(room_name)
        pub_room_name.publish(room_name)

    def callback(self, data):
        print(data.data)
        if (data.data == "cleaning_ended"):
            self.end_of_cleaning = True

    def execute(self, userdata):
        publish_room_name("lazienka")
        while (not self.end_of_cleaning):
            try:
                (trans,rot) = self.tf_listener1.lookupTransform('/odom', '/symbol_1', rospy.Time(0))
                if (trans[0] is not None and trans[1] is not None): 
                    self.obstacle_found = True
                    float_array = Float32MultiArray()
                    float_array.data = trans
                    # float_array.layout
                    pub_obstacle_loc.publish(float_array)
                    rospy.sleep(1)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

        return 'end_of_cleaning'

class AvoidingObstacle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['obstacle_avoided'])

    def execute(self, userdata):
        rospy.loginfo('Executing state AVOIDNG_OBSTACLE')

        #jezeli dojechal do bazy
        return 'obstacle_avoided'

class Docking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['docking_ended'])
        rospy.Subscriber("/RoomCleanerState", String, self.dockCallback)
        self.is_docked = False

    def dockCallback(self, data):
        if (data.data=="cleaning_ended"):
            self.is_docked=True


    def execute(self, userdata):
        rospy.loginfo('Executing state CLEANING')
        publish_room_name("dock")


        while (not self.is_docked):
            rospy.loginfo('DOCKING')

        return 'docking_ended'

def main():
    rospy.init_node('ima_state_machine')
    rate = rospy.Rate(10) # 10hz

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['the_end'])
    
    # Open the container
    with sm_top:

        smach.StateMachine.add('INIT', Init(),
                               transitions={'dock_exists':'CLEANING',
                                            'dock_doesnt_exist':'FIND_DOCK'})
            # Add states to the contakm m iner
        smach.StateMachine.add('CLEANING', Cleaning(), 
                                   transitions={'obstacle_found':'AVOIDNG_OBSTACLE', 
                                                'end_of_cleaning':'DOCKING'})
                                    # transitions={ 'end_of_cleaning':'DOCKING'})

        smach.StateMachine.add('FIND_DOCK', FindDock(), 
                                   transitions={'dock_exists':'CLEANING'})
        smach.StateMachine.add('DOCKING', Docking(), 
                                   transitions={'docking_ended':'the_end'})
        smach.StateMachine.add('AVOIDNG_OBSTACLE', AvoidingObstacle(), 
                                   transitions={'obstacle_avoided':'CLEANING'})

    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    pub_room_name = rospy.Publisher('/room_name_publisher', String, queue_size=10)
    pub_obstacle_loc = rospy.Publisher('/obstacle_loc_publisher', Float32MultiArray ,queue_size=10 )
    main()