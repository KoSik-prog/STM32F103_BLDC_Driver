import serial, io, time, threading, eel, keyboard
import serial.tools.list_ports

runFlag = True

@eel.expose 
def get_python_variable():
    return connect.get_motor_data()

@eel.expose 
def set_motor_position(motorPos):
    connect.set_new_position(motorPos)

def close_callback(route, websockets):
    global runFlag
    
    runFlag = False
    if not websockets:
        exit()

class Connect:  
    motorData = ''
      
    def __init__(self, comPort):
        print("port init")
        ports = serial.tools.list_ports.comports()
        for p in ports:
            print(p.device)
        print(len(ports), 'ports found')
        rxData = ''
        try:
            self.serialConn = serial.Serial(comPort, 115200, timeout=0, parity=serial.PARITY_EVEN, rtscts=1)
        except:
            print("device not found")
            exit()
        
    def read_thread(self):
        while runFlag == True:
            time.sleep(.1)  
            self.rxData = self.readData()
            self.parse_data(self.rxData)  
            
    def get_motor_data(self):
        return self.motorData
    
    def set_new_position(self, newPos):
        pos = '#deg{}0/'.format(newPos)
        self.serialConn.write(str(pos).encode('ascii'))
            
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
        self.eel.init('web', allowed_extensions=['.js', '.html'])
        self.name = 'eel test'
        self.create_window()
        
    def create_window(self):
        eel.start('index.html', size=(800, 1000), block = False, close_callback=close_callback)  
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
    connect = Connect("COM3") 
    main = Main()
    
    readTH = threading.Thread(target=connect.read_thread)
    readTH.start()
    mainTH = threading.Thread(target=main.main_thread)
    mainTH.start()
    
    gui = Gui()