#Run with python3!

import subprocess as sub
import os,datetime

def getrevision():
    # From https://www.raspberrypi-spy.co.uk/2012/09/getting-your-raspberry-pi-revision-number-using-python/
    # Extract board revision from cpuinfo file
    revision = "undefined"
    try:
        f = open('/proc/device-tree/model','r')
        for line in f:
            length=len(line)
            revision = line[0:length-1]
        f.close()
    except:
        revision = input("PI version not detected, enter manually:\n")
    return revision

def getmac():
    # Extract Wlan mac address from sys file
    mac = "00:00:00:00:00:00"
    try:
        f = open('/sys/class/net/wlan0/address','r')
        for line in f:
            length=len(line)
            mac = line[0:length-1]
        f.close()
    except:
        mac = input("MAC address not detected, enter manually:\n")
    return mac

def gethat():
    # Extract HAT id from device-tree
    hat = "undefined"
    try:
        f = open('/proc/device-tree/hat/product','r')
        for line in f:
            length=len(line)
            hat = line[0:length-1]
        f.close()
    except:
        hat = input("HAT not detected, enter manually:\n")
    return hat

def getusb():
    try:
        command = "lsblk -o NAME,SIZE | grep -w sda"
        usb = str(sub.check_output(command, shell=True))
        size=int(len(usb))
        usb=float(usb[size-9:size-4])
        if usb > 128:
            output = "256GB"
        elif usb > 64:
            output = "128GB"
        elif usb > 32:
            output = "64GB"
        elif usb > 16:
            output = "32GB"
        elif usb > 8:
            output = "16GB"
        else:
            output = "USB Memory to small!"
    except:
        output = "No USB memory detected!"
    return output

def getsd():
    try:
        command = "lsblk -o NAME,SIZE | grep -w mmcblk0"
        sd = str(sub.check_output(command, shell=True))
        size=int(len(sd))
        sd=float(sd[size-9:size-4])
        if sd > 128:
            output = "256GB"
        elif sd > 64:
            output = "128GB"
        elif sd > 32:
            output = "64GB"
        elif sd > 16:
            output = "32GB"
        elif sd > 8:
            output = "16GB"
        else:
            output = "SD Memory to small!"
    except:
        output = "Error detecting the SD storage!"
    return output


print("Welcome to the Behaviour Benchmarking Hardware check, please answer the following questions carefully\n")
country = input("In what country are you running this test? (Ex.: CH, CA etc.)\n")
autolab = input("In what autolab are you working? (Ex.: ETHZ, MIT etc.)\n")
db_version = input("According to https://docs.duckietown.org/daffy/opmanual_duckiebot/out/duckiebot_configurations.html, please enter the type of your Duckiebot (DB19, DB18, DB18-Encoder, DB18-Robotarium, DB20, DBv2)\n")
db_sw_version = input("Please enter the branch of the Software your running on your Duckiebot (Master19, daffy, daffy_new_deal etc.)\n")
print("Please follow the following instructions to specify and doublecheck the important components\n")
print("Please visit https://gitlab.com/llingg/behaviour_benchmarking/-/blob/master/hw-checklist.md and follow the instructions of step 1-6\n")
hw_request = input("Did all HW checks pass? [y/n]\n")
if hw_request=='y':
    verdict = "True"
else:
    verdict = "False"
    print("Please note that there is no use in continuing here as the benchmark won't be valid\n")

print("To answer the following questions please refer to https://gitlab.com/llingg/behaviour_benchmarking/-/blob/master/hw-checklist.md\n")

chassis_request = input("Is the duckiebot using the Magician Red Chassis? [y/n]\n")
if chassis_request=='y':
    chassis = "Magician Red Chassis"
else:
    chassis = input("Enter the chassis description:\n")
hostname = os.uname()[1]

date = str(datetime.date.today())

mac = str(getmac())

platform = str(getrevision())

hat_version = str(gethat())

usb_memory = str(getusb())
sd_memory = str(getsd())

bat_request = input("Is the duckiebot using a standard white battery? [y/n]\n")
if bat_request=='y':
    battery = "RAVPOWER RP-PB07"
else:
    battery = input("Enter the battery description:\n")

act_request = input("Is the duckiebot using the yellow motors? [y/n]\n")
if act_request=='y':
    actuation = "DG01D dual-axis drive gear (48:1)"
else:
    actuation = input("Enter the actuation description:\n")

wheel_type = input("What wheel type is mounted on your Duckiebot? (Ex.: Type 1, Type 2, Type 3)\n")

led_type = input("What LED type is mounted on your Duckiebot? (Ex.: Type 1, Type 2, Type 3)\n")

cam_request = input("Is the duckiebot using the Waveshare Raspberry Pi Camera Module Kid 1080P with 160-FOV Fisheye Lens? [y/n]\n")
if cam_request=='y':
    camera = "Waveshare Raspberry Pi Camera"
else:
    camera = input("Enter the camera description:\n")

tester_name = input("Enter your name:\n")

filename = "/data/config/"+country+"_"+autolab+"_"+tester_name+"_"+date+"_"+db_version+"_hardware-compliance.yaml"

f= open(filename,"w+")
f.write("verdict: "+verdict+"\n")
f.write("hostname: "+hostname+"\n")
f.write("db_version: "+db_version+"\n")
f.write("date: "+date+"\n")
f.write("country: "+country+"\n")
f.write("autolab: "+autolab+"\n")
f.write("mac-adress: "+mac+"\n")
f.write("platform: "+platform+"\n")
f.write("hat_version: "+hat_version+"\n")
f.write("usb-memory: "+usb_memory+"\n")
f.write("sd-memory: "+sd_memory+"\n")
f.write("battery: "+battery+"\n")
f.write("actuation: "+actuation+"\n")
f.write("wheel_type: "+wheel_type+"\n")
f.write("camera: "+camera+"\n")
f.write("led_type: "+led_type+"\n")
f.write("tester_name: "+tester_name)
f.close()
