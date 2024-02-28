import serial
import time
import pandas as pd
import threading
import matplotlib.pyplot as plt
import seaborn as sns

### This program is a simple sensor readout function for rapid sensor evaluation for interaction measurements ###
### if a sensor moves over a beacon/landmark it should detect its relative position to the landmark ###
# possible sensors would be a photo sensor that moves over a light source, hall sensor moving close to magnet or similar

# the test setup: a linear lead screw actuator (with encoder) controlled by an arduino through the main thread,
# a linear gauge for reference high precision measurement in its own thread, it saves its distance values to a list
# a sensor that needs to be evaluated by the linear gauge. it has its own thread and list for saving its data

# the setup is for rapid sensor testing, therefore sensors and motor control are not on the same MCU !
# this causes obvious time delays and the only comparable clock is the computer clock that operates all three COMs
# this would be needed for actual precision measurements,
# but for exploration testing this accuracy is good enough for slow motor movements

# All sensor values have timestamps. all lists are merged, sorted and values interpolated according to their timestamps
# the high precision gauge values are plotted against the test sensor value (light, hall, etc...)
# this gives an idea, if the sensor can qualify for high precision relative positioning measurements
# if yes a more optimized test-setup and hardware (MCU, PCB etc...) would be needed

class HaiveEye:
    def __init__(self):
        # we currently have to connect 3 COM ports, 2 for value reading (test sensor and gauge), one for motor control
        self.arduino = serial.Serial(port='COM14', baudrate=115200, timeout=.1)
        self.sensor = serial.Serial(port='COM3', baudrate=115200, timeout=.1)
        self.distance = serial.Serial(port='COM13', baudrate=115200, timeout=.1)
        # some values are for convenience, so we don't have to calculate them later,
        # so some are technically unnecessary.
        self.startTime = 0
        self.roundStart = 0
        self.direction = "calibration"
        self.round = 0
        # events for starting times and motor feedback,
        # sensor values and distance values get their own thread
        # and own lists to avoid deadlocks while writing into the list
        self.events = []
        self.sensorValues =[]
        self.sensorThread = threading.Thread(target=self.readsensor, daemon=True)
        self.distanceValues =[]
        self.distanceThread = threading.Thread(target=self.readDistance, daemon=True)
        self.stepValues = []
        print("initiated")



    #################################### Thread functions ####################################
    ##########################################################################################
    def readsensor(self):
        # this is a thread function. keep reading sensor values and save them
        while True:
            sensorValue = self.sensor.readline().decode('UTF-8').strip()
            self.sensorValues.append([time.time(), round(time.time() - self.startTime, 2), round(time.time() - self.roundStart, 2), self.round, self.direction, "sensorValue", int(sensorValue)])

    def readDistance(self):
        # this is a thread function. keep reading distance values and save them
        while True:
            distanceValue = self.distance.readline().decode('UTF-8').strip()
            self.distanceValues.append(
                [time.time(), round(time.time() - self.startTime, 2), round(time.time() - self.roundStart, 2),
                 self.round, self.direction, "distanceValue", float(distanceValue)])


    ##################################### Move functions #####################################
    ##########################################################################################


    def move(self,value):
        # this function is actually only for manual motor movements, by the user,
        # in case the system needs to be fine-tuned
        self.arduino.write(bytes("M{}".format(value), 'utf-8'))
        print("move {}".format(value))

    def moveForward(self):
        # the distance is very small, this command just moves forward by 2100 steps, which is about 1.2cm
        # this step value was deducted through testing and observation by eye. it takes about 2.5 seconds
        # askForStep() is used to ask the firmware/encoder about the current step count.
        # this is needed to ensure that the motor finished its movements
        currentStep = self.askForStep()
        if currentStep == 0:
            print("move Forward")
            self.direction = "forward"
            self.arduino.write(bytes("M-2100", 'utf-8'))
            self.events.append([time.time(), round(time.time() - self.startTime, 2), round(time.time() - self.roundStart, 2), self.round, self.direction, "eventValue", "moveForward"])
            time.sleep(3)


    def moveBackward(self):
        # the distance is very small, this command just moves backward by 2500 steps, which is about 1cm
        # this step value was deducted through testing and observation by eye
        currentStep = self.askForStep()
        if currentStep == -2100:
            print("move Backward")
            self.direction = "backward"
            self.arduino.write(bytes("M2100", 'utf-8'))
            self.events.append([time.time(), round(time.time() - self.startTime, 2), round(time.time() - self.roundStart, 2), self.round, self.direction, "eventValue", "moveBackward"])
            time.sleep(3)

    def askForStep(self):
        # for security we check the position first by sending "s" to the firmware
        self.arduino.write(bytes("s", 'utf-8'))
        got_position = False
        # note that "s" will give us multiple values back, like acceleration, microsteps and other settings
        # the serial COM communication therefore has a stack of messages which are sent in response to "s"
        # in this program we are only interested in the current step count
        # this is one measurement loop, which we leave when we worked off the stack and "Current Position: xxx" is read
        while not got_position:
            lastMessage = self.arduino.readline().decode('UTF-8')
            if 'Current Position' in lastMessage:
                currentStep = int(lastMessage.split(sep=": ")[-1])
                self.stepValues.append(
                    [time.time(), round(time.time() - self.startTime, 2), round(time.time() - self.roundStart, 2),
                     self.round, self.direction, "stepsValue", int(currentStep)])
                got_position = True
                print("current Step {}".format(currentStep))
        return currentStep

    ################################### Protocol functions ###################################
    ##########################################################################################

    def protocol_1(self,repeats=100):
        # we can call this functions with the repeats-variable or we enable the user interaction below
        # Taking input from user and initiate and start threads
        # repeats = int(input("Enter a number: ")) # enable this for user interaction
        time.sleep(0.05)
        # starting threads
        self.sensorThread.start()
        self.distanceThread.start()

        # start logging events, these are technically not used for the plots, but could be used for debugging
        self.startTime = time.time()
        self.events.append(
            [time.time(), round(time.time() - self.startTime, 2), round(time.time() - self.roundStart, 2), self.round,
             self.direction, "eventValue", "startProtocol"])
        # repeat for many rounds, based on user input or repeats-variable
        for i in range(repeats):
            time.sleep(0.5)
            self.round = i+1
            self.roundStart = time.time()
            print("round {}".format(self.round))
            self.moveForward()
            print("forward done")
            self.moveBackward()
            print("backward done")
            i = i + 1

        self.saveFile()
        print("done")

    # we save the files for later use. We use Pandas to save them as csv dataframes, mostly because it is convenient
    def saveFile(self):
        self.dfEvent = pd.DataFrame(self.events, columns=['timestamp', 'protocolTime', 'roundTime', 'round',
                                                       'direction', 'description', 'Eventsvalue'])
        # saving the dataframe
        self.dfEvent.to_csv(r'C:\Users\David\Documents\protocol1events.csv')
        self.dfsensor = pd.DataFrame(self.sensorValues, columns=['timestamp', 'protocolTime', 'roundTime', 'round',
                                                     'direction', 'description', 'sensorValue'])
        # saving the dataframe
        self.dfsensor.to_csv(r'C:\Users\David\Documents\protocol1sensor.csv')
        self.dfDistance = pd.DataFrame(self.distanceValues, columns=['timestamp', 'protocolTime', 'roundTime', 'round',
                                                           'direction', 'description', 'Distancevalue'])
        # saving the dataframe
        self.dfDistance.to_csv(r'C:\Users\David\Documents\protocol1distance.csv')
        self.dfStep = pd.DataFrame(self.stepValues, columns=['timestamp', 'protocolTime', 'roundTime', 'round',
                                                                     'direction', 'description', 'Stepvalue'])
        # saving the dataframe
        self.dfStep.to_csv(r'C:\Users\David\Documents\protocol1step.csv')

    # plotting the sensor value against the gauge distance
    # there are some minor preprocessings going on here,
    # but basically we just concat the loaded dataframes, sort, interpolate and drop a few columns we don't need
    def plot_dataframes(self):
        df_sensor = pd.read_csv('C:/Users/David/Documents/protocol1sensor.csv')
        df_sensor.drop(["description"], axis=1, inplace=True)
        # double checking what the dataframe looks like
        print(df_sensor.head())
        df_distance = pd.read_csv('C:/Users/David/Documents/protocol1distance.csv')
        df_distance.drop(["description"], axis=1, inplace=True)
        result = pd.concat([df_sensor, df_distance], ignore_index=True, sort=False)
        result.sort_values(by=["timestamp"], inplace=True)
        result.interpolate(method='linear', limit_direction='forward', axis=0, inplace=True)
        df3 = result[(result['direction'] != "calibration")]
        # the next line is technically not needed, but we can actually round up and reduce the accuracy of the gauge
        # it is counterintuitive, but it might help to see the overlap of the trials:
        # the gauge values are so precise (0.01 mm) that they are barely repeating each other in each trial
        # in this case we don't get a standard deviation, which we might want to have

        #df3.round({'Distancevalue': 1, 'sensorValue': 0})
        # print(df3.head())

        # plot with seaborn library, feel free to google "seaborn lineplot" for an example
        # Plot the sensor data for forward and backward direction in the same plot
        sns.set_theme(style="darkgrid")

        sns.lineplot(x="Distancevalue", y="sensorValue", hue="direction", data=df3)
        plt.show()

Eye = HaiveEye()
#Eye.move(5000)
Eye.protocol_1(10)
Eye.plot_dataframes()


