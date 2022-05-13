from tkinter import *
from paho.mqtt import client as mqtt_client
import json

broker = '35.238.225.243'
port = 1883
topic = "api/request"
topic_sub = "Dar/Mal"
# generate client ID with pub prefix randomly
client_id = 'your client id'
username = 'your username'
password = 'your password'
deviceId = "your deviceId"


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Successfully connected to MQTT broker")
        else:
            print("Failed to connect, return code %d", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def publish(client, status):
    # msg = f"messages: {msg_count}"
    msg = "{\"action\":\"command/insert\",\"deviceId\":\"" + deviceId + "\",\"command\":{\"command\":\"LED_control\",\"parameters\":{\"led\":\"" + status + "\"}}}"
    # msg = '{"action":"command/insert","command":{"id":432436060,"command":"LED_control","timestamp":"2021-03-24T00:19:44.418","lastUpdated":"2021-03-24T00:19:44.418","userId":37,"deviceId":"s3s9TFhT9WbDsA0CxlWeAKuZykjcmO6PoxK6","networkId":37,"deviceTypeId":5,"parameters":{"led":"on"},"lifetime":null,"status":null,"result":null},"subscriptionId":1616544981034531}'
    result = client.publish(topic, msg)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")


def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        # print(f"Recieved '{msg.payload.decode()}' from '{msg.topic}' topic")
        y = json.loads(msg.payload.decode())
        temp = str(y["notification"]["parameters"]["temp0"])
        hum = str(y["notification"]["parameters"]["humi0"])
        print("temperature: ", temp, ", humidity:", hum)
        temp_label.config(text=temp + " °C",
                          fg="black")

        hum_label.config(text=hum + "  %",
                         fg="black")

    client.subscribe(topic_sub)
    client.on_message = on_message


window = Tk()
window.title("Control GreenHouse")
window.geometry('405x800')
window.resizable(False, False)
window.configure(bg="white")
canvas = Canvas(window, bg="white", width=395, height=135)
canvas.place(x=0, y=0)
img = PhotoImage(file="logo.png")
canvas.create_image(0, 0, anchor=NW, image=img)

canvas2 = Canvas(window, width=120, height=120)
canvas2.place(x=50, y=165)
img2 = PhotoImage(file="temp2.png")
canvas2.create_image(0, 0, anchor=NW, image=img2)

canvas3 = Canvas(window, width=120, height=120)
canvas3.place(x=50, y=310)
img3 = PhotoImage(file="hum.png")
canvas3.create_image(0, 0, anchor=NW, image=img3)

is_on = False

# Create Label
my_label = Label(window,
                 text="выключено",
                 bg="white",
                 fg="grey",
                 font=("Helvetica", 32))
my_label.place(x=200, y=500)


def switch():
    global is_on
    if is_on:
        on_button.config(image=off)
        my_label.config(text="выключено!",
                        fg="grey")
        print("LED is off now")
        publish(client, "0")
        is_on = False
    else:
        on_button.config(image=on)
        my_label.config(text="включено!",
                        fg="red")
        print("LED is On now")
        publish(client, "1")
        is_on = True


# Define Our Images
on = PhotoImage(file="on1.png")
off = PhotoImage(file="off1.png")

# Create A Button
on_button = Button(window, image=off, bd=0,
                   command=switch)
on_button.place(x=50, y=460)

canvas4 = Canvas(window, bg="white", width=395, height=135)
canvas4.place(x=0, y=700)
img4 = PhotoImage(file="hv2.png")
canvas4.create_image(0, 0, anchor=NW, image=img4)

# Create Label
temp_label = Label(window,
                   text=" °C",
                   bg="white",
                   fg="black",
                   font=("Helvetica", 42))

temp_label.place(x=180, y=190)

# Create Label
hum_label = Label(window,
                  text="  %",
                  bg="white",
                  fg="black",
                  font=("Helvetica", 42))

hum_label.place(x=180, y=325)
client = connect_mqtt()
subscribe(client)
client.loop_start()

window.mainloop()
client.loop_stop()