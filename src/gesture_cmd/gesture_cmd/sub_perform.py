import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_interfaces.srv import ActionTrigger
import DOGZILLALib as dog
from Speech_Lib import Speech
import time
from std_srvs.srv import SetBool
 
class DogPerform(rclpy.node.Node):
  def __init__(self):
      super().__init__('sub_perform')
      self.subscription = self.create_subscription(String, '/dogzilla/pub_gesture/gesture', self.gesture_callback, 1)
      self.dogControl = dog.DOGZILLA()
      self.spe = Speech()
      self.last_gesture = ''
      
      self.declare_parameter('enable', False);
      self.declare_parameter('is_free_running', False);
      self.declare_parameter('cmd_order', [255, 255, 255, 255, 255, 255]);
      
      self.srv = self.create_service(ActionTrigger, 'action_trigger', self.handle_action_trigger)
      self.srv = self.create_service(SetBool, 'en_lookup', self.handle_en_lookup)
       
  def handle_en_lookup(self, request, response):
    print(f"'en_lookup' service called for Enable Lookup: {request.data} ...")
    if request.data == True:
      self.dogControl.attitude("p", -30)    # Lookup
      self.en_lookup = True
    else:
      self.dogControl.attitude("p", 0)    # Reset to normal gesture
      self.en_lookup = False
    return response
      
  def handle_action_trigger(self, request, response):
    print(f"'action_trigger' service called for gesture: {request.gesture} ...")
    self.action_select(request.gesture)
    return response
      
  def perform_action(self, action_code):
    self.spe.void_write(1)
    self.dogControl.action(action_code)
    print(f"Perform: {action_code}...")
    
  def action_select(self, gesture):
    if self.get_parameter('enable').value:
        cmd_order = self.get_parameter('cmd_order').value
        if gesture == 'Zero':
          self.perform_action(cmd_order[0])    
        elif gesture == 'One':
          self.perform_action(cmd_order[1])
        elif gesture == 'Two':
          self.perform_action(cmd_order[2])
        elif gesture == 'Three':
          self.perform_action(cmd_order[3])
        elif gesture == 'Four': 
          self.perform_action(cmd_order[4])
        elif gesture == 'Five': 
          self.perform_action(cmd_order[5])
         
  def gesture_callback(self, msg):
    # action_list = self.get_parameter('enable').value
    if self.get_parameter('is_free_running').value:
      self.action_select(msg.data)
          
    self.last_gesture = msg.data 
    
def main(args=None):
  rclpy.init(args=args)
  dog_perform = DogPerform()
  rclpy.spin(dog_perform)
    
if __name__ == '__main__':
  main()  
  