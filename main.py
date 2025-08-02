import network
import time
from machine import Pin,PWM,I2C
import dht
import ssd1306
import ujson
import utime
from umqtt.simple import MQTTClient
import ntptime
import time

def sync_time():
  try:
    ntptime.settime()
    print("Zeit synchronisiert:", time.localtime())
  except Exception as e:
    print("Fehler bei NTP:", e)


# MQTT Config
MQTT_CLIENT_ID = "sunlu-controller"
MQTT_BROKER    = "mqtt.local"
MQTT_TOPIC     = "sunlu/sensors"
MQTT_STATUS    = "sunlu/status"
#I2C Config
i2c = I2C(0, scl=Pin(22), sda=Pin(21))  # Pins anpassen!
oled = ssd1306.SSD1306_I2C(128, 64, i2c)


# Sensor Pins
SENSOR_PINS = [15, 2, 4, 16]
SENSORS = [dht.DHT22(Pin(pin)) for pin in SENSOR_PINS]
HEATER =  PWM(Pin(25),freq=1000)
def set_heater_power(percent):
    duty = int(percent / 100 * 1023)
    HEATER.duty(duty)
FAN = PWM(Pin(26),freq=1000)
def set_fan_power(percent):
    duty = int(percent / 100 * 1023)
    FAN.duty(duty)

#Fan after run
fan_run_until = 300


#I2C Display
def update_display(temp, heater_power, fan_power, humidity=None, warning=None):
    oled.fill(0)  # Bildschirm löschen

    oled.text("Filament Trockner", 0, 0)
    oled.text("Temp: {:.1f} C".format(temp), 0, 12)
    oled.text("Heizung: {}%".format(int(heater_power)), 0, 24)
    oled.text("Luefter: {}%".format(int(fan_power)), 0, 36)

    if humidity is not None:
        oled.text("Feuchte: {}%".format(int(humidity)), 0, 48)

    if warning:
        oled.text("! WARNUNG !", 0, 56)

    oled.show()

# WiFi Connect
def connect_wifi():
  print("Connecting to WiFi", end="")
  sta_if = network.WLAN(network.STA_IF)
  sta_if.active(True)
  sta_if.connect('Wokwi-GUEST', '')
  while not sta_if.isconnected():
    print(".", end="")
    time.sleep(0.2)
  print(" Connected!")

# MQTT Setup
def connect_mqtt():
  global client
  client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER, keepalive=60)
  client.set_last_will(MQTT_STATUS, "offline", retain=True)
  client.connect()
  client.publish(MQTT_STATUS, "online", retain=True)
  print("MQTT Connected!")

# Safe Sensor Read
def safe_measure(sensor):
  try:
    sensor.measure()
    return {
      "temp": sensor.temperature(),
      "humidity": sensor.humidity()
    }
  except Exception as e:
    print("Sensor error:", e)
    return {
      "temp": None,
      "humidity": None
    }
#Temp over 30degress
def any_sensor_over_threshold(sensors, threshold=30.0):
    for sensor in sensors:
        try:
            sensor.measure()
            if sensor.temperature() > threshold:
                return True
        except:
            pass
    return False

#PID Regler 
class PID:
  def __init__(self, Kp, Ki, Kd, setpoint):
    self.Kp = Kp
    self.Ki = Ki
    self.Kd = Kd
    self.setpoint = setpoint
    self.last_error = 0
    self.integral = 0

  def compute(self, current_value):
    error = self.setpoint - current_value
    self.integral += error
    derivative = error - self.last_error
    self.last_error = error
    output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
    return max(0, min(100, output))  # Begrenze auf 0–100 %
def adapt_pid(pid, current_temp, last_temp):
    delta = abs(current_temp - last_temp)

    # Dynamische Anpassung
    if delta > 2.0:
        pid.Kp = 5.0
        pid.Ki = 0.1
        pid.Kd = 1.0
    elif delta > 1.0:
        pid.Kp = 3.0
        pid.Ki = 0.2
        pid.Kd = 0.5
    else:
        pid.Kp = 1.5
        pid.Ki = 0.3
        pid.Kd = 0.2


def get_average_values(sensors):
    temps = []
    hums = []
    for sensor in sensors:
        try:
            sensor.measure()
            temps.append(sensor.temperature())
            hums.append(sensor.humidity())
        except Exception as e:
            print("Sensorfehler:", e)
    avg_temp = sum(temps) / len(temps) if temps else None
    avg_hum  = sum(hums) / len(hums) if hums else None
    return avg_temp, avg_hum


# Main Loop
connect_wifi()
sync_time()
connect_mqtt()

TARGET_TEMP = 45.0  # Zieltemperatur in °C
pid = PID(Kp=2.0, Ki=0.1, Kd=0.5, setpoint=TARGET_TEMP)

#pid = PID(Kp=1.5, Ki=0.05, Kd=0.3, setpoint=TARGET_HUM)
#power = pid.compute(avg_hum)

oled.fill(0)
oled.text("Starte...", 0, 24)
oled.show()
time.sleep(2)


while True:
  ensure_mqtt = True
  try:
    client.ping()
  except:
    connect_mqtt()


  print("Reading sensors...")
  avg_temp, avg_hum = get_average_values(SENSORS)
  print("🌡 Durchschnitt: Temp = {:.1f}°C | Feuchte = {:.1f}%".format(avg_temp, avg_hum))
  heater_power = pid.compute(avg_temp)
  set_heater_power(heater_power)
  #set_fan_power(power)
  print("🔥 Heizleistung: {:.1f}%".format(heater_power))
  #FAN 
  # Lüfterbedingung prüfen
  sensor_hot = any_sensor_over_threshold(SENSORS)
  if heater_power > 0 or sensor_hot:
    fan_power = 100  # volle Leistung
  else:
    fan_power = 0    # Lüfter aus
  
  set_fan_power(fan_power)





  payload = {
    "timestamp": "{}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(*time.localtime()[:6]),
    "sensors": [safe_measure(s) for s in SENSORS],
    "avg_temp": avg_temp,
    "avg_hum": avg_hum,
    "heater_power": heater_power
  }
  msg = ujson.dumps(payload)
 
  warning = "Ueberhitzung" if avg_temp > 60 else None
  update_display(avg_temp, heater_power, fan_power,avg_hum, warning)


  print("Publishing:", msg)
  client.publish(MQTT_TOPIC, msg)
  time.sleep(5)
