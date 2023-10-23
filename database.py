import pyrebase

class coordinate:
    def __init__(self, name):
        self.name = name
        
        self.x = None
        self.y = None
        self.z = None
    
    def update(self, cord):
        self.x = float(cord['X'])
        self.y = float(cord['Y'])
        self.z = float(cord['Z'])
    
    def show(self):
        if self.x == None:
            print('None')
            return
        print('%.3f | %.3f | %.3f' % (self.x, self.y, self.z))

class database():
    firebaseConfig = {
        'apiKey': "AIzaSyDMp3bTFc7omqzBwBSil1zblGO7QpxJyDc",
        'authDomain': "feedback-d4289.firebaseapp.com",
        'databaseURL': "https://feedback-d4289-default-rtdb.asia-southeast1.firebasedatabase.app",
        'projectId': "feedback-d4289",
        'storageBucket': "feedback-d4289.appspot.com",
        'messagingSenderId': "570628209367",
        'appId': "1:570628209367:web:648bb7a3d00c62af838e92",
        'measurementId': "G-0BZSFJCDBV"
        }


    def __init__(self, user):
        self.username = user
        
        firebase = pyrebase.initialize_app(self.firebaseConfig)
        self.user = firebase.auth().sign_in_anonymous()
        self.database = firebase.database()

        self.acc = coordinate([0, 0, 0])
        self.gyro = coordinate([0, 0, 0])
        self.cmp = 0
        self.yaw = 0
        self.orgHead = None
    
    def get(self):
        data = self.database.child(self.username).get().val()
        # data = self.database.get().val()
        
        self.acc.update(data['Accelerometer'])
        self.gyro.update(data['Gyroscope'])
        self.cmp = float(data['Compass'])
        self.yaw = float(data['Yaw']) 
            
            
        
    def showAll(self):
        print('\nData:')
        print('>> Accelerometer:')
        self.acc.show()
        
        print('>> Gyroscope:')
        self.gyro.show()
        
        print('>> Compass:')
        print(self.cmp)
        
        print('>> Yaw:')
        print(self.yaw)
        
        
        
        
        


    