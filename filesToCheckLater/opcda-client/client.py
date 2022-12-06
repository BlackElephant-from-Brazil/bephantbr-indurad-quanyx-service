import OpenOPC

opc = OpenOPC.client()

# opc.connect('Matrikon.OPC.Simulation')

print(opc.servers())

opc.close()