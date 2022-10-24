import serial, io, time, threading, eel, keyboard, os
import serial.tools.list_ports

runFlag = True

@eel.expose 
def get_python_variable():
    return connect.get_motor_data()

@eel.expose 
def set_motor_position(motorPos):
    connect.set_new_position(motorPos)
    
@eel.expose 
def set_motor_power(motorPower):
    connect.set_new_power(motorPower)
    
@eel.expose 
def set_mode(mode):
    connect.set_mode(mode)

def close_callback(route, websockets):
    global runFlag
    
    runFlag = False
    if not websockets:
        exit()

class Connect:  
    motorData = ''
      
    def __init__(self):
        self.bldcPort = ''
        self.bldcPort = self.find_device()
        if(self.bldcPort == -1 or self.bldcPort == None):
            print('device not found!')
            serial.Serial.close()
            exit()
        rxData = ''
        try:
            self.serialConn = serial.Serial(self.bldcPort, 115200, timeout=0, parity=serial.PARITY_EVEN, rtscts=1)
        except:
            print("device not found")
            exit()
            
    def find_device(self):
        print("searching device...")
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.serialConn = serial.Serial(p.device, 115200, timeout=0, parity=serial.PARITY_EVEN, rtscts=1)
            self.serialConn.write(str("?").encode('ascii'))
            time.sleep(.1)
            self.rxData = self.readData()
            self.serialConn.close()
            if self.rxData.count("$") > 0:
                deviceName = self.rxData[self.rxData.index("$")+1 : self.rxData.index("/")]
                if(deviceName == "BLDCController"):
                    print("device found on {}".format(p.device))
                    return p.device
                    break
                else:
                    return -1
            
        
    def read_thread(self):
        while runFlag == True:
            time.sleep(.1)  
            self.rxData = self.readData()
            self.parse_data(self.rxData)  
            
    def get_motor_data(self):
        return self.motorData
    
    def send_data(self, data):
        self.serialConn.write(str(data).encode('ascii'))
        
    def set_mode(self, mode):
        msg = '#mod{}/'.format(mode)
        self.send_data(msg)
    
    def set_new_position(self, newPos):
        msg = '#deg{}0/'.format(newPos)
        self.send_data(msg)
        
    def set_new_power(self, newPow):
        msg = '#pwr{}/'.format(newPow)
        self.send_data(msg)
            
    def parse_data(self, data):
        if data != None:
            if data.count("#") > 0:
                self.motorData = data[data.index("#")+1 : data.index("/")]
        
    def connect(self): 
        self.serialConn.isOpen()
        
    def readData(self):
        rx = ''
        while self.serialConn.inWaiting() > 0:
            rx += self.serialConn.read(1).decode('utf-8')   
        if rx != '':
            return rx
        
    def __del__(self):
        print("connection destruction")
        self.serialConn.close()
        
class Gui:
    def __init__(self):
        print("GUI init")
        self.eel = eel
        self.eel.init(f'{os.path.dirname(os.path.realpath(__file__))}/web', allowed_extensions=['.js', '.html'])
        self.name = 'eel test'
        self.create_window()
        
    def create_window(self):
        eel.start('index.html', size=(800, 1100), block = False, close_callback=close_callback)  
        while True:
            eel.sleep(.1) 
            
class Main:
    def __init__(self) -> None:
        print("main init")
            
    def main_thread(self):
        global runFlag
        
        while runFlag == True:
            time.sleep(1)
            
if __name__ == "__main__":
    connect = Connect() 
    main = Main()
    
    readTH = threading.Thread(target=connect.read_thread)
    readTH.start()
    mainTH = threading.Thread(target=main.main_thread)
    mainTH.start()
    
    gui = Gui()