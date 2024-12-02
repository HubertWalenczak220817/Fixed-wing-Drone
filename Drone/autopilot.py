import socket
import logging
import math
import time

FG_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
FG_socket.bind(('localhost', 12345))

logger = logging.getLogger(__name__)
console_handler = logging.StreamHandler()

# Create a formatter and add it to the handlers
formatter = logging.Formatter(
"%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)

logging.basicConfig(filename='autopilot.log', level=logging.DEBUG)

def ClampDegrees360(angle):
	angle = angle % 360.0
	if (angle < 0): return angle + 360.0
	return angle
	
	
def ClampDegrees180(angle):
	angle = ClampDegrees360(angle)
	if (angle > 180): angle -= 360
	return angle



class PIDController:
	def __init__(self, kp = 0, ki = 0, kd = 0, Max = 1, Min = -1):
		self._prevError = 0
		self.INTAccum = 0
		self.Kp = kp
		self.Ki = ki
		self.Kd = kd
		self._max = Max
		self._min = Min

        
	def Reset(self):
		self._prevError = self.INTAccum = 0
	
	
	def Compute(self, error, deltaTime):
		logger.debug(f"Computing PID with error: {error} and deltaTime: {deltaTime}")
		self.INTAccum += error * deltaTime
		action = self.Kp * error + self.Ki * self.INTAccum + self.Kd * (error - self._prevError) / deltaTime
		action = action / 200.0
		logger.debug(f"Action = {action}")
		logger.debug(f"_min = {self._min}, _max = {self._max}")
		clamped = max(self._min, min(self._max, action))
		logger.debug(f"Clamped = {clamped}")
		
		if (clamped != action):
			self.INTAccum -= error * deltaTime

		self._prevError = error

		logger.debug(f"PID output: {clamped}")
		return clamped;
	

class Autopilot:
	def __init__(self):
		# Declare targets
		self.AltitudeTarget = 1000
		self.HeadingTarget = 90
		self.RealRollTarget = 0
		self.SpeedTarget = 60
		self.RealVertSpeedTarget = 0
		self.Spd = 0
		
		self.VertSpeedTargetLimit = 15
		self.BankAngle = 45
		self.RollLimit = 45
		
		self.PitchDownLimit = 15
		self.PitchUpLimit = 25
		
		self.throttle = 0.2
		self.elevator = 0
		self.aileron = 0
		self.rudder = 0
		
		# Declare PIDs values
		self.AccKp = 0.5
		self.AccKi = 50
		self.AccKd = 0.020
		self.PitKp = 2.0
		self.PitKi = 1.0
		self.PitKd = 0.005
		self.RolKp = 0.8
		self.RolKi = 0.002
		self.RolKd = 0.1
		self.YawKp = 1.0
		self.YawKi = 0.25
		self.YawKd = 0.02
		
		# Declare PIDs
		self.AccelerationPIDController = PIDController(self.AccKp, self.AccKi, self.AccKd, 1, 0)
		self.PitchPIDController = PIDController(self.PitKp, self.PitKi, self.PitKd, 1, -1)
		self.RollPIDController = PIDController(self.RolKp, self.RolKi, self.RolKd, 1, -1)
		self.YawPIDController = PIDController(self.YawKp, self.YawKi, self.YawKd, 1, -1)
		
		self._initPitchController = True
		self._initRollController = True
		self._initYawController = True
		
		

	def parser(self, data):
		data = str(data[:-2])
		logger.debug(data)	
		data = data.split("\t")
		self.pitch = float(data[0])
		self.roll = float(data[1])
		self.heading = float(data[2])
		self.altitude = float(data[3])
		self.velocity = float(data[4])
		self.latitude = float(data[5])
		self.longitude = float(data[6])
		self.verticalSpeed = float(data[7])
		self.gForceZ = float(data[8])
		logger.info(f"Updated autopilot data: pitch={self.pitch}, roll={self.roll}, altitude={self.altitude}, velocity={self.velocity}")


	def convertAltitudeToVerticalSpeed(self, deltaAltitude, reference = 5):
		logger.debug(f"Converting altitude {deltaAltitude} to vertical speed with reference {reference}")
		if (reference < 2):
			reference = 2;

		if (deltaAltitude > reference or deltaAltitude < -reference):
			return deltaAltitude / reference;
		if (deltaAltitude > 0.1):
			return max(0.05, deltaAltitude * deltaAltitude / (reference * reference));
		if (deltaAltitude < -0.1):
			return min(-0.05, -deltaAltitude * deltaAltitude / (reference * reference));
		return 0;
        
        
        
	def SynchronizePIDs(self):
        
		# synchronize PID controllers
		if (self._initPitchController):
			self.PitchPIDController.INTAccum = self.elevator * 100 / self.PitchPIDController.Ki;

		if (self._initRollController):
			self.RollPIDController.INTAccum = self.aileron * 100 / self.RollPIDController.Ki;

		if (self._initYawController):
			self.YawPIDController.INTAccum = self.rudder * 100 / self.YawPIDController.Ki;

		self._initPitchController = False;
		self._initRollController = False;
		self._initYawController = False;
		
		
	def UpdateDeltaTime(self, deltaTime):
		logger.debug(f"Updating delta time: {deltaTime}")
		self.deltaTime = deltaTime
		
		
	def SpeedHold(self):
		spd = self.velocity;
		CurAcc = (spd - self.Spd) / self.deltaTime;
		self.Spd = spd;

		self.RealAccelerationTarget = (self.SpeedTarget - spd) / 4;
		AErr = self.RealAccelerationTarget - CurAcc;
		self.AccelerationPIDController.INTAccum = max(-1 / self.AccKi, min(self.AccelerationPIDController.INTAccum, 1 / self.AccKi));
		tAct = self.AccelerationPIDController.Compute(AErr, self.deltaTime);

		if (math.isnan(tAct)):
			self.AccelerationPIDController.Reset();
			self.throttle = 0.2

		else:
			self.throttle = tAct
		logger.info(f"Speed control: throttle set to {self.throttle}")


	def AltitudeHold(self):
		deltaAltitude = self.AltitudeTarget - self.altitude
		self.RealVertSpeedTarget = self.convertAltitudeToVerticalSpeed(deltaAltitude, 10)
		self.RealVertSpeedTarget = max(-self.VertSpeedTargetLimit, min(self.RealVertSpeedTarget, self.VertSpeedTargetLimit))
		logger.info(f"Altitude control: RealVertSpeedTarget set to {self.RealVertSpeedTarget}")
		
		
	def VertSpeedHold(self):
		# NOTE: 60-to-1 rule:
		# deltaAltitude = 2 * PI * r * deltaPitch / 360
		# Vvertical = 2 * PI * TAS * deltaPitch / 360
		# deltaPitch = Vvertical / Vhorizontal * 180 / PI
		deltaVertSpeed = self.RealVertSpeedTarget - self.verticalSpeed;
		adjustment = deltaVertSpeed / (self.velocity+0.0000001) * 180 / 3.14;

		self.RealPitchTarget = self.pitch + adjustment;

		self.RealPitchTarget = max(-self.PitchDownLimit, min(self.RealPitchTarget, self.PitchUpLimit));
		
		PitchErr = (self.RealPitchTarget - self.pitch);

		self.PitchPIDController.INTAccum = max(-100 / self.PitKi, min(self.PitchPIDController.INTAccum, 100 / self.PitKi));
		PitchAct = self.PitchPIDController.Compute(PitchErr, self.deltaTime);

		if (math.isnan(PitchAct)):
			self.PitchPIDController.Reset();
		else:
			self.elevator = PitchAct
		logger.info(f"Vertical speed control: elevator set to {self.elevator}")
	
	def RollHold(self):
		self.RealRollTarget = max(-self.BankAngle, min(self.RealRollTarget, self.BankAngle))
		logger.debug(f"RealRollTarget Bank constrained = {self.RealRollTarget}")
		self.RealRollTarget = max(-self.RollLimit, min(self.RealRollTarget, self.RollLimit))
		logger.debug(f"RealRollTarget Roll constrained = {self.RealRollTarget}, roll = {self.roll}")
		RollErr = ClampDegrees180(self.RealRollTarget - self.roll);
		logger.debug(f"RollErr = {RollErr}")


		#self.RollPIDController.INTAccum = min(-100 / self.RolKi, max(self.RollPIDController.INTAccum, 100 / self.RolKi));
		logger.debug(f"RollPIDController.INTAccum = {self.RollPIDController.INTAccum}")
		RollAct = self.RollPIDController.Compute(RollErr, self.deltaTime);
		logger.debug(f"RollAct = {RollAct}")

		if (math.isnan(RollAct)):
			self.RollPIDController.Reset();
		else:
			self.aileron = RollAct
		logger.info(f"Roll control: aileron set to {self.aileron}")
	
	def Drive(self):
		self.SynchronizePIDs()
		self.SpeedHold()
		self.AltitudeHold()
		self.VertSpeedHold()
		self.RollHold()
		return max(0, min(self.throttle*100, 1)), -self.elevator, self.aileron, self.rudder
		


autopilot = Autopilot()
previous_time = time.time()

while True:
	#client_socket, addr = FG_socket.accept()
	data, addr = FG_socket.recvfrom(1024)
	data = data.decode('utf-8')
	
	current_time = time.time()
	delta_time = current_time - previous_time
	previous_time = current_time
	autopilot.UpdateDeltaTime(delta_time)
	
	autopilot.parser(data)
	throttle, elevator, aileron, rudder = autopilot.Drive()
	#MESSAGE = f"{throttle}\t{elevator}\t{aileron}\t{rudder}\n"
	MESSAGE = f"{throttle}\t{elevator}\t{aileron}\n"
	logger.debug(f"Sending to: {addr}")
	logger.debug(f"Message: {MESSAGE}")
	FG_socket.sendto(bytes(MESSAGE, "utf-8"), ("localhost", 12346))
	
	
	
