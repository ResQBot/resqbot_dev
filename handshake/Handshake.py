import serial
import time

__handshake = False

for portNo in range (1, 10): 
    comPort = format("COM{}".format(int(portNo)))
    try:
        chainController = serial.Serial(port= comPort, baudrate = 115200, timeout = 1)
        tx_msg = format("Who are you?\n")
        chainController.write(tx_msg.encode('utf-8'))
        print(comPort)

        time.sleep(0.5)

        rx_msg = chainController.readline().strip().decode("utf-8")
        print(rx_msg)

        if rx_msg == 'ChainController':
            tx_msg = format("Hello ChainController\n")
            chainController.write(tx_msg.encode('utf-8'))
            
            time.sleep(0.5)

            rx_msg = chainController.readline().strip().decode("utf-8")
            print(rx_msg)
            while __handshake == False:
                if rx_msg == 'confirmed':
                    __handshake = True
                else:
                    time.sleep(0.5)
            break
                
        else:
            continue

    except:
        print('Error')


while __handshake:
    mode = input("Enter mode as 1 or 0: ")
    tx_msg = format("P{}\n".format(int(mode)))
    chainController.write(tx_msg.encode('utf-8'))