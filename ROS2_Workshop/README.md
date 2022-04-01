# Workshop Instructions

Today we will be programming the Create3's using ROS2. To help you through this process we have developed a [jupyter notebok](https://jupyter.org/). In order to access this notebook there are a few steps outlined below. 

1. SSH into the Raspberry Pi inside your Create3. The IP Address is at the bottom of the Post It Note infront of you.
2. `Username: ubuntu` `Password: iRobot`
3. Great! Now type the following command into your ssh session 
```bash
ls
```
4. You should see a file called `<ROBOTNAME>.name`, where `<ROBOTNAME>` is replaced with the name of your specific Robot. Make sure you're connected to the correct one!
5. Change into the project directory by typing the following command 
```bash
cd SawyerCode/Create3_Python
```
6. Now start the jupyter notebook server
```bash
jupyter notebook
```
7. You're done with the pi!
8. Now on your laptop you can connect to this server by your web browser. Navigate to the `<IP_ADDRESS>:8888`. Where `<IP_ADDRESS>` is the one used in Step 1. The password is `iRobot`.
9. You should have the directory of your pi infront of you. Click on the `ROS2_Workshop` folder and start on page 00.
