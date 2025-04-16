# ESP32 (IDF). The GPS+GSM+gyroscope alarm tracker with BLE token authtorization & configuration via web page.

# Web pages (activated on ACC ON. Deactiovated on "no activity on web page")
![[photo]](./demo_images/wp1.jpg)
![[photo]](./demo_images/wp2.jpg)

# Hardware design
## with Li-ion 1S. 
AMS1117 LDO has to big voltage drop(1.4v). Mast be changed on an other LDO (dV<0.3V) for using with Li-ion 1S.
![[demo page ]](./demo_images/liion1s.png)
![[demo page ]](./demo_images/liion1s_view.jpg)
## with LiFePo4 2S (final)
More preferable for unattender devices (non flammable).
![[demo page ]](./demo_images/lifepo4_2s_view.jpg)

#current consumption.
* ESP32 + gyroscope - 3.3V 40mA (120mA - BLE scaning)
* GPS neo-6m (+antenne) - 3.3V 50mA (without UNX CFG-RXM Power Save Mode)
* GSM SIM800L - 4.0V 50..100mA (without SIM800L_DTR_GPIO+"AT+CSCLK=1") 

# SMS comamnds
* "I[nfo]" - get the gps state
* "S[tate]" - get the last state
* "T[rack]" - get list of last gps states.
* "C[onf]" - get the current configuration.
* "G[yro]<n>,<n>" - set the gyroscope alarm threshold
* "V[oltage]<n>,<n>" - set the local accumulator voltage alarm threshold
* "A[larm]<0/1>" - disable/enable send alarm sms.
* "B[le]" - get (by sms) list of BLE devices nearby.
* "E[nableBle] <hex1,hex2,..>" - list of authorazed BLE devices
* "H[ang up]0,1" - 0-microphone on. 1-send sms on call.
* Reboot

# SMS async. alarm messages 
* gyroscope alarm (impact|moving|..) (no authorized BLE nearby)
* ACC ON/OFF (no authorized BLE nearby)
* local accumulator voltage (below treshold)
* forwarding incomming sms fom unknown source
