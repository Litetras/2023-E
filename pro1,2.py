import sensor, image, time, pyb,lcd
from pyb import UART,Pin,Timer,Servo
from pid import PID
p_out = Pin('P1', Pin.OUT_PP)#设置p_out为输出引脚
pin1 = Pin('P3', Pin.IN, Pin.PULL_UP)   ##将P3口作为阈值控制口 OUT_PP PULL_NONE
p_out.low()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # 120*160
sensor.set_vflip(True)
sensor.set_hmirror(True)
#sensor.set_auto_exposure(False, \
    #exposure_us = int(19000))# 调整曝光
#lcd.init()
sensor.skip_frames(time=2000)
clock = time.clock()
uart = UART(3, 115200)# p4TX,p5RX
pan_servo=Servo(1)#p7，水平
tilt_servo=Servo(2)#P8，竖直

def data_recv(recv_data):
    head = b'\x53\x5A\x48\x59'
    head_ptr = 0
    head_find_flag = 0
    if recv_data:  # 判断是否有数据,根据接收数据类型修改
        for x in range(0, len(recv_data) - 8):  # 寻找帧头
            if recv_data[x:x+4] == head:
                head_ptr=x  # 记录帧头位置
                head_find_flag=1
                break
        if head_find_flag:  # 找到帧头
            recv_length = recv_data[head_ptr + 4]  # 计算数据包的长度
            if recv_data[head_ptr+recv_length-1] == (sum(recv_data[head_ptr:head_ptr+recv_length-1]) % 256):  # 计算校验和
                cmd = recv_data[head_ptr + 5]  # 获取命令字
                if cmd == 0x02:
                    print(recv_data[head_ptr + 6])
def data_send(cmd , data):#传入数据为bytes类型，
    head = b'\x59\x48\x5A\x53'
    length_data = (0x07 + len(data)).to_bytes(1,'big')
    send = head +length_data+cmd+data
    num = sum(send)
    send = send+(num% 256).to_bytes(1,'big')
    uart.write(send)
    #print(send)

def find_max(blobs):
    max_size=0
    flag=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size and blob[2]*blob[3] <800:
            max_blob=blob
            max_size = blob[2]*blob[3]
            flag=1
    if flag==1:
        return max_blob
    else:
        return None

pan_pid = PID(p=0.40, i=0.02 , imax=90,d=0)
tilt_pid = PID(p=0.40, i=0.02, imax=90,d=0)

pan_servo.calibration(1080,1420,1250)#水平限位
tilt_servo.calibration(1200,1440,1350)#竖直限位
c_pan_servo = 1250
c_tilt_servo = 1350
pan_servo.pulse_width(1250)#水平中点，变小向右 #调用calcuate_PID_X后的实际位置
tilt_servo.pulse_width(1350)#竖直中点，变小向下 #调用calcuate_PID_Y后的实际位置

def calcuate_PID_X (PID_x):
    global c_pan_servo
    c_pan_servo -= PID_x
    return c_pan_servo
def calcuate_PID_Y (PID_y):
    global c_tilt_servo
    c_tilt_servo -= PID_y
    return c_tilt_servo


def led():
    p_out.high()#设置p_out引脚为高
    time.sleep(50)

buzzer_flag  = 0
while(True):
    clock.tick()
    while pin1.value() == 0:
        pass
    #img = sensor.snapshot().binary([(75, 100, -20, 20, -20, 20)])#.erode(1)
    img = sensor.snapshot().binary([(35, 100, 18, 98, -10, 43)]).dilate(1)
    #lcd.display(img)
    if pan_servo.pulse_width()<1300:
        blobs = img.find_blobs([(100,100)], area_threshold=1,roi=(7,2,209,236))
    else :
        blobs = img.find_blobs([(100,100)], area_threshold=1)
    for blob in blobs:
        if blob.area() <800:
            max_blob = find_max(blobs)
            if find_max(blobs)!= None:
                pan_error = max_blob.cx()-153#手调
                tilt_error = max_blob.cy()-122#手调
                if pan_error<=6 and tilt_error<=6 and pan_error>=-6 and tilt_error>=-6:
                    p_out.high()
                else:
                    p_out.low()
                img.draw_rectangle(max_blob.rect(),color=(255,0,0))
                img.draw_cross(max_blob.cx(), max_blob.cy(),color=(255,0,0))
                pan_output=pan_pid.get_pid(pan_error,1)/2
                tilt_output=tilt_pid.get_pid(tilt_error,1)

                pan_servo.pulse_width(calcuate_PID_X(int(pan_output)))#水平中点，变小向右
                tilt_servo.pulse_width(calcuate_PID_Y(int(tilt_output)))#竖直中，变小向下





