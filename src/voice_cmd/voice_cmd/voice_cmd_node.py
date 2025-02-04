import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_interfaces.srv import ActionTrigger
import DOGZILLALib as dog
from Speech_Lib import Speech
from std_srvs.srv import SetBool

import rclpy.parameter
from rcl_interfaces.msg import SetParametersResult

import speech_recognition as sr
import sounddevice  # for not showing ALSA error message
import re
import threading

robotCmd = ["Reset", "Sit Down", "Handshake", "Mark Time", "3 Axis", "Turn Around"];
enSpeech = ["reset", "sit down", "handshake", "mark time", "stretch", "turn around"];
thSpeech = ["เริ่มต้นใหม่", "นั่ง", "ขอมือ", "ย่ำเท้า", "บิดขี้เกียจ", "หมุนตัว"];
jpSpeech = ["リセット", "座る", "握手", "マークタイム", "ストレッチ", "回っている"];

class VoiceCmd(rclpy.node.Node):
    def __init__(self):
        super().__init__('voice_cmd')
        self.dogControl = dog.DOGZILLA()
        # Speech synthesis
        self.spe = Speech()    
                
        self.declare_parameter('set_robot_mic', False);
        self.declare_parameter('set_sr_language', 'en-US');
        self.add_on_set_parameters_callback(self.parameter_callback)
            
        self.srv = self.create_service(ActionTrigger, '/voice_cmd/action_trigger', self.handle_action_trigger)
        
        # Start the speech recognition thread
        self.speech_thread = threading.Thread(target=self.speech_recognition_loop)
        self.speech_thread.daemon = True  # Ensures the thread exits when the main program ends
        self.speech_thread.start()     
        
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'set_robot_mic':
                if param.value:
                    self.get_logger().info(f'Change mic to robot')
                else:
                    self.get_logger().info(f'Change mic to Foxglove')

        return SetParametersResult(successful=True)
             
    def speech_recognition_loop(self):
        
        def search_string_by_array(sentence: str, search_array: list[str]) -> list[int]:
            # Create a regex pattern by joining the search_array elements with '|'
            pattern = re.compile('|'.join(map(re.escape, search_array)), re.IGNORECASE)  # 're.IGNORECASE' makes it case-insensitive

            # Return the indices of the elements in search_array that match the sentence
            return [
                index for index, word in enumerate(search_array)
                if pattern.search(word) and re.search(re.escape(word), sentence, re.IGNORECASE)
            ]         
           
        while rclpy.ok():   
        
            if self.get_parameter('set_robot_mic').value:
                # obtain audio from the microphone
                r = sr.Recognizer()
                        
                with sr.Microphone() as source:
                    self.get_logger().info("Adjusting for ambient noise... Please wait.")
                    r.adjust_for_ambient_noise(source, duration=1)
        
                    self.get_logger().info("Listening for your input...")
                    audio = r.listen(source, timeout=None, phrase_time_limit=5)
                
                # recognize speech using Google Speech Recognition
                try:
                    language = self.get_parameter('set_sr_language').value
                    lang_speech = {"en-US": enSpeech, "th-TH": thSpeech, "ja-JP": jpSpeech}.get(language, [])
                                
                    recognized_text = r.recognize_google(audio, language=language)
                    self.get_logger().info("You said: " + recognized_text)
                    search_results = search_string_by_array(recognized_text, lang_speech)
                    if search_results:
                        speech = robotCmd[search_results.pop()]     
                        self.get_logger().info("Your command is: " + speech)               
                        self.action_select(speech)
                    else:
                        self.get_logger().warning("No matching command found.")
                except sr.UnknownValueError:
                    self.get_logger().warning("Could not understand the speech.")
                except sr.RequestError as e:
                    self.get_logger().error("Could not request results from Google Speech Recognition service; {0}".format(e))

    def handle_action_trigger(self, request, response):
        self.get_logger().info(f"'action_trigger' service called for voice command: {request.gesture} ...")
        self.action_select(request.gesture)
        return response  

    def perform_action(self, action_code):
        self.spe.void_write(38)     # Say "OK"
        self.dogControl.action(action_code)
        self.get_logger().info(f"Perform: {action_code}...")    
    
    def action_select(self, voice_cmd):
        if voice_cmd == 'Reset':
            self.perform_action(255)    
        elif voice_cmd == 'Sit Down':
            self.perform_action(12)
        elif voice_cmd == 'Handshake':
            self.perform_action(19)
        elif voice_cmd == 'Mark Time':
            self.perform_action(5)
        elif voice_cmd == '3 Axis': 
            self.perform_action(10)
        elif voice_cmd == 'Turn Around': 
            self.perform_action(4)      
        
def main(args=None):
    rclpy.init(args=args)
    dog_perform = VoiceCmd()        
    rclpy.spin(dog_perform)
    
if __name__ == '__main__':
  main()  