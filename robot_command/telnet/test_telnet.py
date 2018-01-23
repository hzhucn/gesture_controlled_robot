import telnetlib

ip, port = 'khepera2.smart.metz.supelec.fr', 4100

command = "D,-200,200"

try: 
    tn = telnetlib.Telnet(ip, port, timeout=1)
    while True:
        tn.write(command.encode('ascii')+ b"\r")
        output = tn.read_some()
        print(output)
    tn.close()
except Exception as e:
        print("Impossible d'envoyer un message: {}".format(e))

