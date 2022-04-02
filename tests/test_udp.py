import time
import unitree_udp

udp = unitree_udp.UnitreeUDPWrapper()
udp_cmd = []

while True:
    time1 = time.time()
    udp_state = udp.recv()
    print(udp_state)
    print("length: ", len(udp_state))
    udp.send(udp_cmd);
    print("time elapsed: ", (time.time()-time1)*1000)
    time.sleep(0.5)