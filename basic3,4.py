import sensor, image, time, pyb,lcd
from pyb import UART,Pin,Timer,Servo
from pid import PID
pin1 = Pin('P1', Pin.IN, Pin.PULL_DOWN)   ##将P1口作为阈值控制口 OUT_PP PULL_NONE
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA) # 120*160
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_auto_exposure(False, \
    exposure_us = int(8200))# 调整曝光
sensor.skip_frames(time=2000)
clock = time.clock()
lcd.init()
uart = UART(3, 115200)# p4为TX,p5为RX

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

pan_pid = PID(p=0.45, i=0.01 , imax=20,d=0)
tilt_pid = PID(p=0.45, i=0.01 , imax=20,d=0)





def ctrl_servo(x,y):
    data = x.to_bytes(2,'big')+y.to_bytes(2,'big')
    data_send(b'\x01',data)




#记录矩形位置
count=0
p_check=None
while(count<50 or p_check==None):
    img = sensor.snapshot().binary([(0,20)]).erode(1)
    lcd.display(img)
    clock.tick()
    for rect in img.find_rects(threshold = 90000,roi=(4,3,152,112)):#滤波用的roi
        for p in rect.corners():
            img.draw_circle(p[0], p[1], 5, color = (0, 255, 0))
            p_check=p
        pos=rect.corners()
    count+=1
    time.sleep_ms(10)

global a,b,c,d
a=pos[0]
b=pos[1]
c=pos[2]
d=pos[3]



def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob




def mov(x,y):
    while(True):
        clock.tick()

        img = sensor.snapshot().binary([(35, 100, 18, 98, -10, 43)]).dilate(2)
        blobs = img.find_blobs([(100, 100)], area_threshold=1)#LAB用100，100
        aa=img.draw_line(a[0]*2,a[1]*2,b[0]*2,b[1]*2,color = (255, 255, 255))
        bb=img.draw_line(b[0]*2,b[1]*2,c[0]*2,c[1]*2,color = (255, 255, 255))
        cc=img.draw_line(c[0]*2,c[1]*2,d[0]*2,d[1]*2,color = (255, 255, 255))
        dd=img.draw_line(d[0]*2,d[1]*2,a[0]*2,a[1]*2,color = (255, 255, 255))
        if blobs:
            while pin1.value() == 1:
                pass
            max_blob = find_max(blobs)
            img.draw_rectangle(max_blob.rect(),color=(255,0,0)) # rect
            img.draw_cross(max_blob.cx(), max_blob.cy(),color=(255,0,0)) # cx, cy
            #lcd.display(img.draw_cross(max_blob.cx(), max_blob.cy(),color=(255,0,0)))###########
            pan_error = max_blob.cx()-x*2
            tilt_error = max_blob.cy()-y*2
            pan_output=pan_pid.get_pid(pan_error,1)/2
            tilt_output=tilt_pid.get_pid(tilt_error,1)
            ctrl_servo(calcuate_PID_X(int(pan_output)),calcuate_PID_Y(int(tilt_output)))#水平中点，变小向右#竖直中，变小向下
            if pan_error<=4 and tilt_error<=3 and pan_error>=-4 and tilt_error>=-3:
                break


c_pan_servo = 1500#调用calcuate_PID_X后的实际位置 p7，水平
c_tilt_servo = 1200#调用calcuate_PID_Y后的实际位置 P8，竖直
#pan_servo.calibration(1420,1700,1500)#水平限位
#tilt_servo.calibration(1000,1400,1200)#竖直限位
def calcuate_PID_X (PID_x):
    global c_pan_servo
    c_pan_servo += PID_x
    return c_pan_servo
def calcuate_PID_Y (PID_y):
    global c_tilt_servo
    c_tilt_servo += PID_y
    return c_tilt_servo



ctrl_servo(1500,1200)

sensor.set_pixformat(sensor.RGB565) # RGB565模式
sensor.set_framesize(sensor.QVGA) # 120*160分辨率
sensor.skip_frames(time=500)


def find_point(x1,y1,x2,y2,num):
    step = 1/num
    for i in [i * 0.05 for i in range(21)]:
        mov(int(x1 + (x2 - x1) * step * i),int( y1 + (y2 - y1) * step * i))


while(1):
    find_point(a[0],a[1],d[0],d[1],1)
    find_point(d[0],d[1],c[0],c[1],1)
    find_point(c[0],c[1],b[0],b[1],1)
    find_point(b[0],b[1],a[0],a[1],1)
#找四边形函数需要滤波————————————————————————————————+++++++++++++++++++++++++





