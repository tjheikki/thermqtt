#!/usr/bin/python

import serial
import pprint
import re
import sys
import time
import paho.mqtt.client as mqtt
import pprint as pp

import thermiq_regs

MQTT_HOST = "192.168.0.30"
MQTT_PORT = 1883
MQTT_TOPIC = "thermia"
SERIAL_DEVICE = "/dev/thermiaSerial2"
SERIAL_BPS = 9600
SERIAL_TIMEOUT = 0.1
SERIAL_BUF = 256 #Serial buffer size
regs = thermiq_regs.reg_id

def listRegs():
  """
  Lists all registers by printing
  @return none
  """
  for k,v in regs.items():
    reg = v[0][1:]
    if reg.isdigit():
      reg = int(reg, 16)
      print (k + " : " + str(reg))

def getReg(name):
  """
  Get register number by name
  @param name string  Name of the register, or reg number as string or int
  @return int         Register number, negative if couldn't find
  """
  ret = -1
  if type(name) == int:
    ret = name
  elif name.isdigit():
    ret = int(name)
  else:
    try:
      ret = int(regs[name][0][1:], 16)
    except:
      print ("No register name " + str(name))
  return ret

def filterSerOut(inV):
  """
  Filter unwanted serial stuff like "OK", "" or "^at" from serial data
  @param inV array of strings  Serial data to be filtered
  @return    array of strings  Filtered data
  """
  out = []
  for i in inV:
    if i != "OK" and i != "" and not re.match("^at",i):
      out.append(i)
  return out

def procSerOut(inV):
  """
  Process serial output data to an array of UTF-8 strings without unnecessary overhead
  @param inV string            Input serial data
  @return    array of strings  Cleaned serial data rows as an array
  """
  out = inV.decode("utf-8")
  out = re.split("[\r\n]+", out)
  out = filterSerOut(out)
  return out

def int2hex(inV,bytes = 2):
  """
  Format integer to hexadecimal string of <bytes> chars, padded with zeroes
  @param inV   int     Input number to be formatted as hex
  @param bytes int     Number of characters in the output string, zero padded from the left
  @return      string  Hex string
  """
  if isinstance(inV, int):
    return hex(inV)[2:].zfill(bytes)
  else:
    return inV

def ver():
  """
  Show version of the serial client
  @return string  Result of "ati" serial command
  """
  ser.write(b'ati\n')
  ser.flush()
  out = procSerOut(ser.read(SERIAL_BUF))[0]
  return out

def serRead(reg, count = 5):
  reg = getReg(reg)
  if count == 0:
    return "[reg, None, int(reg, 16), None]"
  regH = int2hex(reg)
  ser.write(('atr' + regH + '\r\n').encode())
  serOut = procSerOut(ser.read(SERIAL_BUF))
  try:
    [r, v] = serOut[0].split("=")
  except:
    return serRead(reg, count - 1)
  return [int(r, 16), int(v, 16)]

def serWrite(reg, val):
  reg = getReg(reg)
  reg = int2hex(reg)
  val = int2hex(val, 4)
  ser.write(('atw' + reg + val + '\r\n').encode())
  out = procSerOut(ser.read(SERIAL_BUF))[0]
  return out

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(MQTT_TOPIC)

# callback for when a PUBLISH message is received from the server
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))

args = len(sys.argv)

if args < 2:
  print("thermqtt <reg> (<val>)")
  print("thermqtt list")
  print("thermqtt mqtt")
elif args == 2:
  if sys.argv[1] == "mqtt":
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT, 60)
    client.loop_start()
    client.publish(MQTT_TOPIC, "Hello, MQTT!")
  elif sys.argv[1] == "list":
    listRegs()
  else:
    reg = sys.argv[1]
    ser = serial.Serial(SERIAL_DEVICE, SERIAL_BPS, timeout=SERIAL_TIMEOUT)
    r = serRead(reg)
    print (str(r[0]) + ": " + str(r[1]))
    ser.close()
else:
  reg = sys.argv[1]
  val = sys.argv[2]
  ser = serial.Serial(SERIAL_DEVICE, SERIAL_BPS, timeout=SERIAL_TIMEOUT)
  if val[0] == '-' or val[0] == '+':
    r = serRead(reg)
    if r[3] != None:
      val = (-1 if val[0] == '-' else +1) * int(val[1:]) + r[3]
    else:
      exit(1)
  else:
    val = int(val)
  serWrite(reg, val)
  time.sleep(0.1)
  r = serRead(reg)
  print (str(r[0]) + ": " + str(r[1]))
  ser.close()
