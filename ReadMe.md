# *serial_ros*


# Change the serial device mode
## Temporary:
1. get the device ID, such as /dev/ttyUSB0
```
ls /dev/ | grep ttyUSB
```
2. set the mode
```
sudo chmod a+rw /dev/ttyUSB0
```
## Forever
1. creat the mode rule file
```
sudo vim -p etc/udev/rules.d/70-ttyUSB.rules
```
2. add these config parameters to the file
```
KERNEL=="ttyUSB*", OWNER="root", GROUP="root", MODE="0666" 
```
3. save the file. 
4. log out and in. Or reboot.

# clone this reposity to your workspace and build, set the serial port and baudrate to the launch file, start to work.
```
git clone https://github.com/guozhenglong/serial_ros.git
```

## about the float data transmission
In the serial write node, I make a tricks transform float data to int data. That make process of decodeing the data more simply. 
`(int) = (double)*10000` 
In the serial read node,
`(double) = (int)/10000`

# welcomt to report your question about this reposity. Email: guozhenglong.cn@gmail.com

   
