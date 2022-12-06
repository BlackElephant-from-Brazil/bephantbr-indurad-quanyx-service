import time


from opcua import Server
NODE_ID = '67d797049560f130d6f4bbd383e89845071a3d3c63194a11f88f4668502d0b22'

def serve():
    # setup our server
    server = Server()
    server.set_endpoint("opc.tcp://0.0.0.0:4840/freeopcua/server")

    # setup our own namespace, not really necessary but should as spec
    uri = "http://examples.freeopcua.github.io"
    idx = server.register_namespace(uri)

    objects = server.get_objects_node()

    # populating our address space
    myobj = objects.add_object(idx, "MyObject")
    myvar = myobj.add_variable(idx, "MyVariable", 250)
    myvar.set_writable()    # Set MyVariable to be writable by clients

    # starting!
    server.start()

    count = 0
    while True:
        time.sleep(2)
        count += 1
        myvar.set_value(count)

        
    
serve()