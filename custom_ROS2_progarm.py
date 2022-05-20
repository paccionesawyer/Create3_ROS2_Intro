#!/usr/bin/python3

from asyncio.log import logger
from io import StringIO

import rclpy
from rclpy.node import Node

import logging

logging.basicConfig(level=logging.INFO)


class ROS2_program_creator():
    def __init__(self, fileName="Custom_ROS2.py") -> None:
        self.fileName = fileName
        
        sub_prompt = "Enter topics you want to subscribe to seperated by commas: "
        pub_prompt = "Enter topics you want to publish to seperated by commas: "

        self.sub_topics = self.get_user_list(sub_prompt)
        self.pub_topics = self.get_user_list(pub_prompt)

        logging.debug(f"Sub Topics = {self.sub_topics}\n Pub Topics = {self.pub_topics}")

        self.topic_dict = self.get_topic_dict()
        self.name = input("What would you like the new Node to be called?: ")

    def get_user_list(self, prompt: str = "Please enter a list of items") -> list:
        '''
        Prompt the user and return the list of topics you want to subscribe to
        Expects input as topic names separated by commas
        '''

        list_str = input(prompt)

        clean_list = [item.strip() for item in list_str.split(sep=",")]
        logging.info(f'The topics entered for prompt "{prompt}" are... \n {clean_list}')

        return clean_list

    def get_topic_list(self) -> list:
        '''
        Get the ros2 topics currently exposed the network, as a list of tuples
        '''

        rclpy.init()
        node_dummy = Node("_ros2cli_dummy_to_show_topic_list")
        topic_list = node_dummy.get_topic_names_and_types()
        node_dummy.destroy_node()
        rclpy.shutdown()
        logging.info(f'The Topic and Messages found are:\n {topic_list}')

        return topic_list

    def get_topic_dict(self) -> dict:
        '''
        Convert the list of ros2 topics to a dictionary 
        '''

        topic_list = self.get_topic_list()
        topic_dict = {}

        for topic, msgs in topic_list:
            topic_dict[topic] = msgs[0]

        logging.debug(f'The Topic and Messages found are:\n {topic_dict}')

        return topic_dict

    def create_file(self) -> StringIO:
        '''
        Create the new custom program
        '''

        with open(self.fileName, 'w') as f:
            self.add_standard_imports(f)
            self.add_message_imports(f)
            self.create_class(f)
            self.create_main(f)

    def add_standard_imports(self, newFile):
        '''
        The essential imports any python ROS2 program will need to have
        '''

        newFile.write('import rclpy\nfrom rclpy.node import Node\nfrom rclpy.qos import qos_profile_sensor_data\n\n')

    def add_message_imports(self, newFile):
        '''
        Given the topics, import the messages that are used on those topics.
        '''

        all_topics =  self.sub_topics + self.pub_topics

        for topic in all_topics:
            if topic not in self.topic_dict:
                logging.error(f"A topic not found on the network was passed as an argument: {topic}")
                raise ValueError(f'No topic named {topic}')
            else:
                msg_path = self.topic_dict[topic]
                split_path = msg_path.split('/')
                msg_type = split_path[-1]
                path = ""
                for i in split_path[:-1]:
                    path += i + '.'
                path = path[:-1]

                import_statement = f'from {path} import {msg_type}'
                newFile.write(import_statement+'\n')
                
                logging.info(f'For topic {topic}, import is {import_statement}')

        newFile.write('\n')

    def create_class(self, newFile):
        '''
        Create the class definition for our custom Node
        '''

        newFile.write(f"class {self.name.capitalize()}(Node):\n\n")
        self.create_init(newFile)
        self.create_callbacks(newFile)
            
    def create_init(self, newFile):
        '''
        Create the init function for the custom Node class
        '''

        newFile.write("\tdef __init__(self, namespace: str = ''):\n")
        newFile.write(f"\t\tsuper().__init__('{self.name.lower()}')\n\n")

        for i, sub in enumerate(self.sub_topics):
            msg_type = self.topic_dict[sub].split('/')[-1]
            callback_function = f"self.callback_{sub.replace('/', '_')}"
            newFile.write(f"\t\tself.sub_{sub.replace('/', '_')} = self.create_subscription({msg_type}, namespace + '{sub}', {callback_function}, qos_profile_sensor_data)\n")

        newFile.write('\n\n')

        for i, pub in enumerate(self.pub_topics):
            msg_type = self.topic_dict[pub].split('/')[-1]
            newFile.write(f"\t\tself.pub_{pub.replace('/', '_')} = self.create_publisher({msg_type}, namespace + '{pub}', 10)\n")

        newFile.write('\n')

    def create_callbacks(self, newFile):
        '''
        Each topic you are subscribed to will have a different callback function
        '''

        for sub in self.sub_topics:
            msg_type = self.topic_dict[sub].split('/')[-1]
            newFile.write(f"\tdef callback_{sub.replace('/', '_')}(self, msg: {msg_type}):\n")
            logger_msg = "f'I heard: {str(msg)}; On topic " + sub + "'"
            newFile.write(f"\t\tself.get_logger().info({logger_msg})\n\n")

    def create_main(self, newFile):
        '''
        Create a main function that will spin the custom Node
        '''

        newFile.write(f'def main(args=None):\n\trclpy.init(args=args)\n\n\t{self.name.lower()} = {self.name.capitalize()}()\n\n\ttry:\n\t\trclpy.spin({self.name})\n\texcept KeyboardInterrupt:\n\t\tprint("Caught Keyboard Interrupt")\n\tfinally:\n\t\tprint("Done")\n\t\t{self.name}.destroy_node()\n\t\trclpy.shutdown()\n\n')

        newFile.write('if __name__ == "__main__":\n\tmain()')

if __name__ == "__main__":
    
    t = ROS2_program_creator()
    t.create_file()